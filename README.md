# LilyGO T-SIM7000G + MQTT + Owner Detect + Home Assistant intergrate

**BEFORE CONTINUE**, in order to use the board in battery powered mode, you need first to read the following about an issue of unexpected shutdown when in battery powered.
Fix is very easy and need to connect Vbat- to GND, i use a 26awg cable.
`https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/issues/65`

### - This project hasn't tested yet using other esp32 boards

## How this project works:
- It is usually in deep sleep mode and wakes up when a vibration occurs using a SW-420 sensor(not tested using other sensor yet, like MPU6050).
- After the wake up, a ble scan takes place to detect the presence of a **BLE keyfob** based on its MAC adress(e.g. beacon or Bluetooth device).
- Sending data through **MQTT** over GPRS/LTE about the BLE keyfob status(found/not_found) and GPS cordinates as well as speed, altitude, gps accuracy and modem informations.
- Stop to obtain **GPS coordinates** when no motion detected for a specific period of time(default is 5 min) and goes back to deep sleep mode again.

---

**Before use** this project you need to create your own include/config.h file using based on include/config.example.h file pattern. 
To flash the code you can use the **Platformio** extension in **VS Code**.

- **[Visual Studio Code](https://code.visualstudio.com/)**  
- **[PlatformIO IDE](https://platformio.org/install/ide?install=vscode)**

---

## Home Assistant 
**NOTE**: File mqtt_tracer.yaml has been created for Home Assistant using the MQTT itergration. 
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
    - button.esptracer_reboot 

> **button.esptracer_reboot** only works if ESP board is awake**

---

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

## Battery tests

Below are the results from several runtime tests of the LilyGO T-SIM7000G operating in **battery-powered mode**.

| Test # | Mode / Functionality | Battery Capacity (mAh) | Runtime (hours) | Notes |
|:------:|----------------------|:----------------------:|:----------------|:------|
| 1 | Continuous GPS tracking (LTE ON) | 3000 | -- | --- |
| 2 | GPS + MQTT updates every 5 min | 3000  |  -- | --- |
| 3 | GPS + MQTT updates every 30 min | 3000 | -- | --- |
| 4 | Deep sleep only (no updates) | 3000 | -- | --- |

> **Note:** Runtime values are approximate and may vary depending on signal strength, temperature, and board revision.

---

## Libraries Used

- TinyGSM by Volodymyr Shymanskyy
- PubSubClient by Nick O'Leary

