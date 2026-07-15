# LilyGO T-SIM7000G + MQTT + Owner Detect + Home Assistant intergrate

**BEFORE CONTINUE**, in order to use the board in battery powered mode, you need first to read the following about an issue of unexpected shutdown when in battery powered.
Fix is very easy and need to connect Vbat- to GND, i use a 26awg cable.
`https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/issues/65`

### - This project hasn't tested yet using other esp32 boards

## How this project works:
- It is usually in deep sleep mode and wakes up when a vibration occurs using a MPU6050's sensor INT pin.
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
**NOTE**: File mqtt_esptracer.yaml has been created for Home Assistant using the MQTT itergration. 
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
    - sensor.esptracer_device_state
    - device_tracker.esptracer_gps_tracker
    - binary_sensor.esptracer_keyfob_connected
    - binary_sensor.esptracer_device_sleeping
    - button.esptracer_reboot
    - button.esptracer_stop_alarm
    - button.esptracer_power_off

> **button.<--->**  entities only works if ESP board is awake**

---

## Hardware Components

| Component | Description |
|------------|-------------|
| **TTGO T-SIM7000G** | ESP32 board with integrated SIM7000G (GSM/LTE/GNSS) modem |
| **MPU6050 sensor** | Detects movement to trigger wake-up |
| **SW-420 Motion Sensor** | Detects motion to trigger wake-up |
| **BLE Keyfob / Beacon** | The Bluetooth device to be detected |
| **SIM Card** | Provides GPRS data connection |
| **GPS Antenna** | Required for accurate location acquisition |
| **LiPo Battery** | 3.7V rechargeable battery (connected via JST port) |

---

## Antenna / Sensor EMI Considerations
1. Maintain physical separation between the cellular antenna and the motion sensor.
The SIM7000G's LTE/GSM antenna emits power in bursts during transmission (TX), which can induce false triggers on the SW-420's DO pin if placed too close.
2. Use twisted-pair or shielded cable for the motion sensor's DO signal line.
If the sensor cable needs to be lengthened to achieve sufficient separation from the antenna, avoid single unshielded wire. Twisted-pair or shielded cable significantly reduces the risk of EMI coupling into the signal line, especially over longer runs.
3. If close proximity is unavoidable in the final enclosure/PCB layout, apply secondary mitigation:

    1. Add a small decoupling capacitor (e.g., 100nF ceramic) across VCC-GND on the SW-420 module, placed close to the module itself, to filter minor power-line spikes.
    2. Route the SW-420's ground connection carefully — avoid large ground loops near the antenna, and keep the ground path as short/direct as possible to the common ground.

--

## Battery tests

Below are the results from several runtime tests of the LilyGO T-SIM7000G operating in **battery-powered mode**.

| Test # | Mode / Functionality | Battery Capacity (mAh) | Runtime (hours) | Notes |
|:------:|----------------------|:----------------------:|:----------------|:------|
| 1 | Continuous GPS tracking (LTE ON) | 3000 | -- | --- |
| 2 | GPS + MQTT updates every 10 seconds | 3000  |  -- | --- |
| 3 | GPS + MQTT updates every 30 min | 3000 | -- | --- |
| 4 | Deep sleep | 3000 | 18072 | Based on LilyGO statistics |

> **Note:** Runtime values are approximate and may vary depending on signal strength, temperature, and board revision.
> T-SIM7000G-S3-Standard DeepSleep Current dynamic changes Min:59uA , Max273uA ,Avg:166uA

---

## Libraries Used

- TinyGSM by Volodymyr Shymanskyy
- PubSubClient by Nick O'Leary

## TODO list

- Using a cell phone's Bluetooth as a keyfob
- Home Assistant keyfob alert automation
- Design a 3D printed case
- Energy consumption optimization (sendInterval according to speed)
- Geofence and SMS/Telegram notification
- Backup mode commands via SMS without the need for an internet connection
- Configure device via MQTT topic(keyfob MAC address, apn, server IP, etc)
- OTA updates