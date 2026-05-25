# WiFi Remote ID Spoofer & Simulator

![WiFi Remote ID Spoofer & Simulator Logo](spoof.png)

**Disclaimer:** This software is intended for educational and research purposes only...

**Disclaimer:** This software is intended for educational and research purposes only. It is your responsibility to ensure compliance with all local, national, and international laws and regulations regarding radio transmissions, privacy, and aviation safety. The authors and distributors of this software are not liable for any misuse or legal violations that may result. Always obtain proper authorization before transmitting any signals or simulating devices in real-world environments.

## Overview

This project provides:
- **Firmware** for ESP32/ESP32-S3 microcontrollers capable of spoofing WiFi-based Remote ID broadcasts (both NDP/NAN and legacy vendor IE frames).
- **Flask-based Web UI** to control the drone simulation, including map-based flight path design, real-time playback controls, pilot location setting, MAC override, and USB port management.
- **Python simulator** that feeds JSON payloads over serial to the firmware, enabling live control of drone and pilot position, speed, and Remote ID metadata.

## Features

- **Dynamic MAC Spoofing**: Auto-generates a randomized suffix after a fixed `60:60:1F` prefix on each boot; user can override via Web UI.
- **Map-based Flight Planner**: Click to drop waypoints; waypoints are highlighted (start = lime green, end = hot purple), with optional loop-back indicator.
- **Real-time Playback**: Play, Pause (orange highlight), and Stop (red highlight) controls. Pause retains current position; Stop halts all serial transmissions.
- **Pilot Location Control**: Separate control box to select and lock pilot coordinates on the map.
- **USB Serial Management**: Auto-detects and displays connection status; immediate red/green updates on connect/disconnect.
- **JSON RPC Protocol**: Structured payloads include fields for `mac`, `drone_lat`, `drone_long`, `drone_altitude`, `pilot_lat`, `pilot_long`, `basic_id`, `status`, and more.

## Ethics & Legality

- **Compliance First**: Before deploying or testing, verify that spoofing WiFi signals is legal in your jurisdiction. Many countries have strict regulations around radio emissions and aviation safety.
- **Responsible Use**: This tool should never be used to hinder, confuse, or endanger actual aircraft operations, law enforcement, or public safety.
- **Non-endorsement**: The authors do not endorse any illicit use. You assume all risks and legal responsibilities.

## Getting Started

### Hardware Requirements
To use this project, you will need the Seeed XIAO ESP32-S3 microcontroller, which features:
- Dual-core processor
- WiFi and Bluetooth capabilities
- Compact size for easy integration

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/esp-wifi-remoteid-spoofer.git
   cd esp-wifi-remoteid-spoofer
   ```
2. **Install Dependencies**
   ```bash
   pip install -r requirements.txt
   ```
3. **Configure & Run**
   - Launch the Flask app: `python mapper.py`
   - Select your USB serial port on the initial screen.
   - Use the Web UI to design flight paths and control playback.

### Firmware Upload
To upload the firmware to the Seeed XIAO ESP32-S3:
1. Connect the XIAO ESP32-S3 to your computer via USB.
2. Open the Arduino IDE and select the appropriate board and port.
3. Upload the firmware code to the device.

### Running the Web Simulator
To run the web simulator:
1. Ensure the XIAO ESP32-S3 is connected and the firmware is uploaded.
2. Open a web browser and navigate to `http://localhost:5000`.
3. Use the Web UI to start simulating drone operations.

## License

Distributed under the MIT License. See `LICENSE` for details.
