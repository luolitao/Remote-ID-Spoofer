#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include "esp_netif.h"
#include "esp_event.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include <cstring>  // for memset
#include <ArduinoJson.h>
#include <Arduino.h>  // for millis() and Serial
#include "esp_system.h"  // for esp_random()

// === User‑configurable spoof MAC ===
// To override the random MAC, set CONFIG_SPOOF_MAC to a 17‑char "XX:XX:XX:XX:XX:XX" string.
static const char* CONFIG_SPOOF_MAC = "60:60:1f:d3:B2:6a"; 

// Last-received Remote ID data
static char    g_basic_id[ODID_ID_SIZE+1] = "";
static double  g_drone_lat = 0.0, g_drone_lon = 0.0;
static int     g_drone_alt = 0;
static double  g_pilot_lat = 0.0, g_pilot_lon = 0.0;
static bool    g_has_data  = false;
static bool    broadcastEnabled = true;

// Dynamic override for source MAC from JSON
static bool    g_dynamic_override = false;
static uint8_t g_override_src_mac[6] = {0};

// Optional override for the source MAC in the beacon frame:
// Set USE_OVERRIDE_MAC to true and OVERRIDE_SRC_MAC to your desired MAC bytes.
static const bool USE_OVERRIDE_MAC = false;
static const uint8_t OVERRIDE_SRC_MAC[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };

static const char *BEACON_SSID = "ESP_SPOOF";
static const size_t BEACON_SSID_LEN = 9;
static const uint16_t BEACON_INTERVAL = 100;
static const uint8_t SEND_COUNTER = 0;

// Build and inject binary ODID messages as a Vendor IE
static void inject_vendor_ie(const char *basic_id,
                             double drone_lat, double drone_lon,
                             int drone_alt,
                             double pilot_lat, double pilot_lon) {
    // Initialize UAS data struct
    ODID_UAS_Data uas;
    memset(&uas, 0, sizeof(uas));

    // Populate BasicID
    odid_initBasicIDData(&uas.BasicID[0]);
    uas.BasicID[0].UAType = ODID_UATYPE_OTHER;
    uas.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
    strncpy(uas.BasicID[0].UASID, basic_id, ODID_ID_SIZE);
    uas.BasicID[0].UASID[ODID_ID_SIZE] = '\0';
    uas.BasicIDValid[0] = 1;

    // Populate Location
    odid_initLocationData(&uas.Location);
    uas.Location.Latitude    = drone_lat;
    uas.Location.Longitude   = drone_lon;
    uas.Location.AltitudeGeo = drone_alt;
    uas.LocationValid        = 1;

    // Populate System (Operator)
    odid_initSystemData(&uas.System);
    uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS;
    uas.System.OperatorLatitude     = pilot_lat;
    uas.System.OperatorLongitude    = pilot_lon;
    uas.SystemValid                 = 1;

    // choose source MAC: dynamic override takes priority
    uint8_t ap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
    uint8_t *src_mac = ap_mac;
    if (g_dynamic_override) {
        src_mac = g_override_src_mac;
    }

    // Build a full beacon frame
    uint8_t frame_buf[512];
    int frame_len = odid_wifi_build_message_pack_beacon_frame(
        &uas,
        (char*)src_mac,
        BEACON_SSID,
        BEACON_SSID_LEN,
        BEACON_INTERVAL,
        SEND_COUNTER,
        frame_buf,
        sizeof(frame_buf)
    );
    if (frame_len > 0) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame_buf, frame_len, false);
        Serial.printf("Beacon tx err=%d, len=%d\n", err, frame_len);
    } else {
        Serial.printf("Beacon build failed: %d\n", frame_len);
    }
    return;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting AP with Vendor IE...");
    // Determine default override MAC: config takes priority, else random 60:60:1f:XX:YY:ZZ
    if (CONFIG_SPOOF_MAC && strlen(CONFIG_SPOOF_MAC) == 17) {
        unsigned int b[6];
        if (sscanf(CONFIG_SPOOF_MAC, "%02x:%02x:%02x:%02x:%02x:%02x",
                   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
            for (int i = 0; i < 6; ++i) {
                g_override_src_mac[i] = (uint8_t)b[i];
            }
            g_dynamic_override = true;
            Serial.printf("Using configured spoof MAC %s\n", CONFIG_SPOOF_MAC);
        }
    } else {
        uint32_t rnd = esp_random();
        g_override_src_mac[0] = 0x60;
        g_override_src_mac[1] = 0x60;
        g_override_src_mac[2] = 0x1f;
        g_override_src_mac[3] = (rnd      ) & 0xFF;
        g_override_src_mac[4] = (rnd >>  8) & 0xFF;
        g_override_src_mac[5] = (rnd >> 16) & 0xFF;
        g_dynamic_override   = true;
        Serial.printf("Using randomized spoof MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
            g_override_src_mac[0], g_override_src_mac[1], g_override_src_mac[2],
            g_override_src_mac[3], g_override_src_mac[4], g_override_src_mac[5]);
    }

    // Initialize TCP/IP network interface and default event loop
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    // Create default Wi-Fi AP network interface
    esp_netif_create_default_wifi_ap();
    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    // Configure AP settings
    wifi_config_t ap_config = { 0 };
    strncpy((char*)ap_config.ap.ssid, BEACON_SSID, BEACON_SSID_LEN);
    ap_config.ap.ssid_len = BEACON_SSID_LEN;
    ap_config.ap.channel = BEACON_INTERVAL / 1; // use channel 100/TU mapping or set explicitly
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ap_config.ap.max_connection = 4;
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &ap_config) );
    // Start Wi-Fi
    ESP_ERROR_CHECK( esp_wifi_start() );
    Serial.println("ESP32 AP started on channel " + String(ap_config.ap.channel));
}

void loop() {
    static char json_buf[512];
    static size_t json_idx = 0;
    static unsigned long lastTx = 0;
    const unsigned long TX_INTERVAL = 500; // ms

    // 1) Read incoming JSON
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || json_idx >= sizeof(json_buf)-1) {
            json_buf[json_idx] = '\0';
            json_idx = 0;
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, json_buf);
            if (!err) {
                // Update globals
                const char* id = doc["basic_id"] | "";
                strncpy(g_basic_id, id, ODID_ID_SIZE);
                g_basic_id[ODID_ID_SIZE] = '\0';
                g_drone_lat  = doc["drone_lat"]      | 0.0;
                g_drone_lon  = doc["drone_long"]     | 0.0;
                g_drone_alt  = doc["drone_altitude"] | 0;
                g_pilot_lat  = doc["pilot_lat"]      | 0.0;
                g_pilot_lon  = doc["pilot_long"]     | 0.0;
                g_has_data   = true;
                Serial.println("JSON updated");

                // Check for dynamic MAC override
                if (doc.containsKey("mac")) {
                    const char* jsonMac = doc["mac"] | "";
                    if (strlen(jsonMac) == 17) {
                        unsigned int b[6];
                        if (sscanf(jsonMac, "%02x:%02x:%02x:%02x:%02x:%02x",
                                   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
                            for (int i = 0; i < 6; ++i) g_override_src_mac[i] = (uint8_t)b[i];
                            g_dynamic_override = true;
                        }
                    } else {
                        // malformed or empty override disables override
                        g_dynamic_override = false;
                    }
                }
                // Check for stop command or empty path to disable broadcasting
                const char* action = doc["action"] | nullptr;
                if (action) {
                    if (strcmp(action, "stop") == 0) {
                        // Stop all broadcasting
                        broadcastEnabled = false;
                        g_has_data = false;
                        Serial.println("STOP received: broadcasts disabled");
                    }
                    else if (strcmp(action, "start") == 0) {
                        // Restart broadcasting on new mission
                        broadcastEnabled = true;
                        g_has_data = true;
                        Serial.println("START received: broadcasts enabled");
                    }
                }
                // If mission path is empty array, also stop
                if (doc.containsKey("path")) {
                    JsonArray path = doc["path"].as<JsonArray>();
                    if (path && path.size() == 0) {
                        g_has_data = false;
                    }
                }
            } else {
                Serial.print("JSON parse error: ");
                Serial.println(err.c_str());
            }
        } else {
            json_buf[json_idx++] = c;
        }
    }

    // 2) Periodically transmit using last JSON data
    // only transmit if broadcasting is enabled and we have data
    if (!broadcastEnabled || !g_has_data) {
        delay(10);
        return;
    }
    if (millis() - lastTx >= TX_INTERVAL) {
        inject_vendor_ie(
            g_basic_id,
            g_drone_lat, g_drone_lon,
            g_drone_alt,
            g_pilot_lat, g_pilot_lon
        );
        lastTx = millis();
        Serial.println("Beacon injected");
    }
}