# LilyGO T-SIM7000G + MQTT + Owner Detect + Home Assistant intergrate
# This project uses the LilyGO T-SIM7000G ESP32 board for a DIY GPS Tracker with owner detection using a BLE keyfob.

## Overview
NOTE: File mqtt_tracer.yaml has been created for Home Assistant use with the mqtt itergration. 
- Go to Home Assistant config folder and in configuration.yaml add the line: mqtt: !include mqtt_esptracer.yaml
- Add the file config/mqtt_esptracer.yaml.
- Restart Home Assistant.
- Go to Setting->Devies & services->Add itergration->mqtt.
- A device ESPTracer will now appear showing the following entites: 
    - sensor.esptracer_accuracy
    - sensor.esptracer_altitude
    - sensor.esptracer_battery_voltage
    - sensor.esptracer_battery_level
    - sensor.esptracer_speed
    - sensor.esptracer_signal_quality
    - sensor.esptracer_modem_info
    - device_tracker.esptracer_gps_tracker
    - binary_sensor.esptracer_keyfob_connected

How this project works:
- It is usually in deep sleep mode and wakes up when a vibration occurs using a SW-420 sensor.
- After the wake up a ble scan takes place to detect the presence of a **BLE keyfob** based on its MAC adress(e.g. beacon or Bluetooth device).
- Sending data through **MQTT** over GPRS/LTE about the BLE tag detection(found/not_found) and GPS cordinates such as speed, altitude, gps accuracy and modem informations.
- Stop to obtain **GPS coordinates** when no motion detected for a specific period of time(default is 5 min) and goes back to deep sleep mode again.


## Hardware Components

| Component | Description |
|------------|-------------|
| **TTGO T-SIM7000G** | ESP32 board with integrated SIM7000G (GSM/LTE/GNSS) modem |
| **SW-420 sensor** | Detects vibration or movement to trigger wake-up |
| **BLE Keyfob / Beacon** | The Bluetooth device to be detected |
| **SIM Card** | Provides GPRS data connection |
| **GPS Antenna** | Required for accurate location acquisition |
| **LiPo Battery** | 3.7V rechargeable battery (connected via JST port) |

---

## Pin Connections

| TTGO Pin | Description |
|-----------|-------------|
| 25 | DTR (modem sleep control) |
| 26 | RX modem |
| 27 | TX modem |
| 4  | Power Key (PWRKEY of SIM7000) |
| 12 | LED status indicator |
| 35 | ADC for battery voltage measurement |
| 32 | Input from ball switch (interrupt wake-up) |

---

## Software

### Libraries Used

- **TinyGSM** – communication with SIM7000G modem  
- **PubSubClient** – MQTT client

