# This project uses the LILYGO T-SIM7000G ESP32 board for a DIY GPS Tracker with owner detection using a BLE Tag keyfob.

## Overview

How this project works:
- It is usually in deep sleep mode and wakes up when a vibration occurs using a SW-420 sensor.
- After the wake up a ble scan takes place to detect the presence of a **BLE tag** based on its MAC adress(e.g. beacon or Bluetooth device),
- Sending data through **MQTT** over GPRS/LTE about the BLE tag detection(found/not_found) and GPS cordinates,
- Stop to obtain **GPS coordinates** when no motion detected for a specific period of time and goes back to deep sleep mode again.


## Hardware Components

| Component | Description |
|------------|-------------|
| **TTGO T-SIM7000G** | ESP32 board with integrated SIM7000G (GSM/LTE/GNSS) modem |
| **SW-420 sensor** | Detects vibration or movement to trigger wake-up |
| **BLE Tag / Beacon** | The Bluetooth device to be detected |
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


| Topic              | Payload Format                                        |
| ------------------ | ----------------------------------------------------- |
| `vehicle/keyfob`   | `{ "status": "found" }` ή `{ "status": "not_found" }` |
| `vehicle/location` | `{ "lat": 37.9838, "lng": 23.7275 }`                  |

