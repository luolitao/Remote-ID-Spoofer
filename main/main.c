#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "cJSON.h"

#include "opendroneid.h"
#include "odid_wifi.h"

static const char *TAG = "REMOTE_ID";

// Configuration from Kconfig
#ifdef CONFIG_SPOOF_MAC_ADDRESS
static const char* CONFIG_SPOOF_MAC = CONFIG_SPOOF_MAC_ADDRESS;
#else
static const char* CONFIG_SPOOF_MAC = "";
#endif

#ifdef CONFIG_BEACON_SSID
static const char *BEACON_SSID = CONFIG_BEACON_SSID;
#else
static const char *BEACON_SSID = "ESP_SPOOF";
#endif

#ifdef CONFIG_BEACON_CHANNEL
static const uint8_t BEACON_CHANNEL = CONFIG_BEACON_CHANNEL;
#else
static const uint8_t BEACON_CHANNEL = 6;
#endif

#ifdef CONFIG_TX_INTERVAL_MS
static const unsigned long TX_INTERVAL_MS = CONFIG_TX_INTERVAL_MS;
#else
static const unsigned long TX_INTERVAL_MS = 500;
#endif

static const uint16_t BEACON_INTERVAL = 100;
static uint8_t send_counter = 0;

// Last-received Remote ID data
static char g_basic_id[ODID_ID_SIZE+1] = "";
static double g_drone_lat = 0.0, g_drone_lon = 0.0;
static int g_drone_alt = 0;
static double g_pilot_lat = 0.0, g_pilot_lon = 0.0;
static bool g_has_data = false;
static bool broadcast_enabled = true;

// Dynamic override for source MAC from JSON
static bool g_dynamic_override = false;
static uint8_t g_override_src_mac[6] = {0};

// Mutex for protecting shared data
static SemaphoreHandle_t data_mutex = NULL;

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

    // Choose source MAC: dynamic override takes priority
    uint8_t src_mac[6];
    if (g_dynamic_override) {
        memcpy(src_mac, g_override_src_mac, 6);
    } else {
        esp_wifi_get_mac(WIFI_IF_AP, src_mac);
    }

    // Build a full beacon frame
    uint8_t frame_buf[512];
    
    // Calculate actual SSID length
    size_t ssid_len = strlen(BEACON_SSID);
    if (ssid_len > 32) ssid_len = 32;
    
    int frame_len = odid_wifi_build_message_pack_beacon_frame(
        &uas,
        (char*)src_mac,
        BEACON_SSID,
        ssid_len,
        BEACON_INTERVAL,
        send_counter,
        frame_buf,
        sizeof(frame_buf)
    );
    
    // Increment send counter for next transmission
    send_counter++;
    
    if (frame_len > 0) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_AP, frame_buf, frame_len, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Beacon tx failed: %s (len=%d)", esp_err_to_name(err), frame_len);
        } else {
            ESP_LOGD(TAG, "Beacon transmitted, len=%d, counter=%d", frame_len, send_counter - 1);
        }
    } else {
        ESP_LOGE(TAG, "Beacon build failed: %d", frame_len);
    }
}

// Parse MAC address string to bytes
static bool parse_mac_address(const char *mac_str, uint8_t *mac_bytes) {
    if (strlen(mac_str) != 17) {
        return false;
    }
    
    unsigned int b[6];
    if (sscanf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
        for (int i = 0; i < 6; ++i) {
            mac_bytes[i] = (uint8_t)b[i];
        }
        return true;
    }
    return false;
}

// Initialize spoof MAC address
static void init_spoof_mac(void) {
    // Check if configured MAC is valid
    if (CONFIG_SPOOF_MAC && strlen(CONFIG_SPOOF_MAC) == 17) {
        if (parse_mac_address(CONFIG_SPOOF_MAC, g_override_src_mac)) {
            g_dynamic_override = true;
            ESP_LOGI(TAG, "Using configured spoof MAC: %s", CONFIG_SPOOF_MAC);
            return;
        }
    }
    
    // Generate random MAC with prefix 60:60:1f
    uint32_t rnd = esp_random();
    g_override_src_mac[0] = 0x60;
    g_override_src_mac[1] = 0x60;
    g_override_src_mac[2] = 0x1f;
    g_override_src_mac[3] = (rnd      ) & 0xFF;
    g_override_src_mac[4] = (rnd >>  8) & 0xFF;
    g_override_src_mac[5] = (rnd >> 16) & 0xFF;
    g_dynamic_override = true;
    
    ESP_LOGI(TAG, "Using randomized spoof MAC: %02x:%02x:%02x:%02x:%02x:%02x",
        g_override_src_mac[0], g_override_src_mac[1], g_override_src_mac[2],
        g_override_src_mac[3], g_override_src_mac[4], g_override_src_mac[5]);
}

// Process received JSON data
static void process_json(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error");
        return;
    }

    // Lock mutex before updating shared data
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update basic ID
        cJSON *basic_id_item = cJSON_GetObjectItem(root, "basic_id");
        if (basic_id_item && cJSON_IsString(basic_id_item)) {
            strncpy(g_basic_id, basic_id_item->valuestring, ODID_ID_SIZE);
            g_basic_id[ODID_ID_SIZE] = '\0';
        }

        // Update drone position
        cJSON *drone_lat = cJSON_GetObjectItem(root, "drone_lat");
        if (drone_lat && cJSON_IsNumber(drone_lat)) {
            g_drone_lat = drone_lat->valuedouble;
        }
        
        cJSON *drone_long = cJSON_GetObjectItem(root, "drone_long");
        if (drone_long && cJSON_IsNumber(drone_long)) {
            g_drone_lon = drone_long->valuedouble;
        }
        
        cJSON *drone_alt = cJSON_GetObjectItem(root, "drone_altitude");
        if (drone_alt && cJSON_IsNumber(drone_alt)) {
            g_drone_alt = (int)drone_alt->valuedouble;
        }

        // Update pilot position
        cJSON *pilot_lat = cJSON_GetObjectItem(root, "pilot_lat");
        if (pilot_lat && cJSON_IsNumber(pilot_lat)) {
            g_pilot_lat = pilot_lat->valuedouble;
        }
        
        cJSON *pilot_long = cJSON_GetObjectItem(root, "pilot_long");
        if (pilot_long && cJSON_IsNumber(pilot_long)) {
            g_pilot_lon = pilot_long->valuedouble;
        }

        g_has_data = true;
        ESP_LOGI(TAG, "JSON data updated");

        // Check for dynamic MAC override
        cJSON *mac_item = cJSON_GetObjectItem(root, "mac");
        if (mac_item && cJSON_IsString(mac_item)) {
            if (parse_mac_address(mac_item->valuestring, g_override_src_mac)) {
                g_dynamic_override = true;
                ESP_LOGI(TAG, "MAC override applied: %s", mac_item->valuestring);
            } else {
                g_dynamic_override = false;
                ESP_LOGW(TAG, "Invalid MAC format in JSON");
            }
        }

        // Check for action commands
        cJSON *action_item = cJSON_GetObjectItem(root, "action");
        if (action_item && cJSON_IsString(action_item)) {
            if (strcmp(action_item->valuestring, "stop") == 0) {
                broadcast_enabled = false;
                g_has_data = false;
                ESP_LOGI(TAG, "STOP received: broadcasts disabled");
            } else if (strcmp(action_item->valuestring, "start") == 0) {
                broadcast_enabled = true;
                g_has_data = true;
                ESP_LOGI(TAG, "START received: broadcasts enabled");
            }
        }

        // Check for empty path
        cJSON *path_item = cJSON_GetObjectItem(root, "path");
        if (path_item && cJSON_IsArray(path_item)) {
            if (cJSON_GetArraySize(path_item) == 0) {
                g_has_data = false;
                ESP_LOGI(TAG, "Empty path received");
            }
        }

        // Release mutex
        xSemaphoreGive(data_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire data mutex");
    }

    cJSON_Delete(root);
}

// Simple UART receive task using getchar
static void uart_receive_task(void *pvParameters) {
    char line_buf[512];
    size_t line_idx = 0;

    while (1) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || line_idx >= sizeof(line_buf) - 1) {
                line_buf[line_idx] = '\0';
                
                if (line_idx > 0) {
                    process_json(line_buf);
                }
                
                line_idx = 0;
            } else {
                line_buf[line_idx++] = (char)c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// Beacon transmission task
static void beacon_task(void *pvParameters) {
    TickType_t last_tx_time = xTaskGetTickCount();
    const TickType_t tx_interval_ticks = pdMS_TO_TICKS(TX_INTERVAL_MS);
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Only transmit if broadcasting is enabled and we have data
        if (broadcast_enabled) {
            bool has_data_local = false;
            char basic_id_local[ODID_ID_SIZE+1] = "";
            double drone_lat_local = 0.0, drone_lon_local = 0.0;
            int drone_alt_local = 0;
            double pilot_lat_local = 0.0, pilot_lon_local = 0.0;
            
            // Lock mutex to read shared data
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                has_data_local = g_has_data;
                if (has_data_local) {
                    strncpy(basic_id_local, g_basic_id, ODID_ID_SIZE);
                    basic_id_local[ODID_ID_SIZE] = '\0';
                    drone_lat_local = g_drone_lat;
                    drone_lon_local = g_drone_lon;
                    drone_alt_local = g_drone_alt;
                    pilot_lat_local = g_pilot_lat;
                    pilot_lon_local = g_pilot_lon;
                }
                xSemaphoreGive(data_mutex);
            }
            
            if (has_data_local && ((current_time - last_tx_time) >= tx_interval_ticks)) {
                inject_vendor_ie(
                    basic_id_local,
                    drone_lat_local, drone_lon_local,
                    drone_alt_local,
                    pilot_lat_local, pilot_lon_local
                );
                last_tx_time = current_time;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// WiFi initialization
static void wifi_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default Wi-Fi AP network interface
    esp_netif_create_default_wifi_ap();
    
    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set Wi-Fi mode to AP+STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    // Configure AP settings
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = BEACON_CHANNEL,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };
    
    // Calculate actual SSID length and copy safely
    size_t ssid_len = strlen(BEACON_SSID);
    if (ssid_len > sizeof(ap_config.ap.ssid) - 1) {
        ssid_len = sizeof(ap_config.ap.ssid) - 1;
    }
    memcpy(ap_config.ap.ssid, BEACON_SSID, ssid_len);
    ap_config.ap.ssid[ssid_len] = '\0';
    ap_config.ap.ssid_len = ssid_len;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started on channel %d with SSID: %s", BEACON_CHANNEL, BEACON_SSID);
}

void app_main(void) {
    ESP_LOGI(TAG, "Remote ID Spoofer starting...");
    
    // Create mutex for protecting shared data
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return;
    }
    
    // Initialize spoof MAC
    init_spoof_mac();
    
    // Initialize WiFi
    wifi_init();
    
    // Create UART receive task
    xTaskCreate(uart_receive_task, "uart_rx", 4096, NULL, 5, NULL);
    
    // Create beacon transmission task
    xTaskCreate(beacon_task, "beacon_tx", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System initialized successfully");
}