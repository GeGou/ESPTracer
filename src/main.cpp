#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "modemManager.h"
// #include "MPU6050.h"
#include <BLEDevice.h>


TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

// ==== BLE Key Fob ====
BLEScan* pBLEScan;
bool keyFobFound = false;

// ==== MPU6050 ====
// MPU6050 mpu;
// #define MPU_INT_PIN 34  // Πρέπει να είναι RTC_GPIO για wakeup
// volatile bool motionDetected = false;


// ==== Tracking ====
unsigned long sendInterval = 10000; // κάθε 10s
unsigned long lastSend = 0;
unsigned long lastMotion = 0;
const unsigned long motionTimeout = 5 * 60 * 1000UL; // 5 λεπτά

bool firstBoot = true;

// --- FUNCTIONS ---
float ReadBatteryVoltage();
int BatteryPercent(float voltage);

// ------------------------------------------------------

void connectToMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  // mqttClient.setCallback(callback);

  while (!mqttClient.connected()) {
    
    Serial.println("Connection to MQTT Broker ...");
    if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(MQTT_TOPIC_KEYFOB);
      mqttClient.subscribe(MQTT_TOPIC_LOC);
      mqttClient.subscribe(MQTT_TOPIC_BAT);
      mqttClient.subscribe(MQTT_TOPIC_MODEM);
    } else {
      Serial.print("Failed to connect to MQTT broker. Error: ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void sendLocation(float lat, float lng, float alt=0, float speed=0, float accuracy=0) {
  String payload = "{\"latitude\":" + String(lat, 6) + ",\"longitude\":" + String(lng, 6) + ",\"altitude\":" + 
    String(alt, 2) + ",\"speed\":" + String(speed, 2) + ",\"gps_accuracy\":" + String(accuracy, 2) + "}";
  mqttClient.publish(MQTT_TOPIC_LOC, payload.c_str());
  Serial.println("📡 Sent: " + payload);
}

void sendKeyFobStatus(bool found) {
  String payload = "{\"status\":\"" + String(found ? "found" : "not_found") + "\"}";
  mqttClient.publish(MQTT_TOPIC_KEYFOB, payload.c_str());
  Serial.println("🔵 BLE key fob: " + String(found ? "found" : "not_found"));
}

void sendBatteryStatus() {
  float voltage = ReadBatteryVoltage();
  int percent = BatteryPercent(voltage);

  String payload = "{\"battery\": " + String(voltage, 2) + ", \"batteryLevel\": " + String(percent) + "}";
  mqttClient.publish(MQTT_TOPIC_BAT, payload.c_str());
  Serial.println("Battery voltage: " + String(voltage, 2) + " V (" + String(percent) + "%)");    
}

void sendModemStatus() {
  String modemInfo = modem.getModemInfo();
  String signalQuality = String(modem.getSignalQuality());

  String payload = "{\"modemInfo\": \"" + modemInfo + "\", \"signalQuality\": " + signalQuality + "}";
  mqttClient.publish(MQTT_TOPIC_MODEM, payload.c_str());
  Serial.println("Modem Info: " + modemInfo);    
  Serial.println("Signal Quality: " + signalQuality);    
}

// ------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("ESPTracer starting...");

  // Αν ΞΥΠΝΗΣΕ από motion
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Motion detected – wakeup event!");
  } else {
    Serial.println("Normal boot");
  }

  // Set LED OFF
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Init modem
  modemPowerOn();

  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(5000); // Wait for serial to initialize

  Serial.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to initialize modem, attempting to continue without modem...");
  }

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }

  delay(1000);

  // Connect to network
  Serial.print(F("Connecting to "));
  Serial.print(APN);
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println(" fail");
    checkModemStatus();
    Serial.println("signal quality: " + String(modem.getSignalQuality()));
    delay(10000);   // Wait 10s and try again
    return;
  }

  Serial.println(" success");
  Serial.print("Local IP: ");
  Serial.println(modem.getLocalIP());

  // Check GPRS connection
  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
  } else {
    Serial.println("GPRS not connected");
  }
  
  // Enable GPS
  GPSTurnOn();
  delay(1000);

  // Connect MQTT
  connectToMQTT();
  delay(1000);

  // === BLE key fob scan - Every time ESP wakes up ===
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);  // Set scan interval to 100ms
  pBLEScan->setWindow(99);   // Set scan window to 99ms (less or equal to setInterval value)

  BLEScanResults results = pBLEScan->start(4, false);
  keyFobFound = false;
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.getAddress().toString() == KEYFOB_MAC_ADDRESS && device.getRSSI() > -80) {
      keyFobFound = true;
    }
  }
  sendKeyFobStatus(keyFobFound);

  // Battery status every time ESP wakes up
  sendBatteryStatus();

  // Modem status every time ESP wakes up
  sendModemStatus();

  lastMotion = millis();
}

void loop() {

  // === GPS ===
  if (firstBoot || millis() - lastSend >= sendInterval) {
    firstBoot = false;
    lastSend = millis();

    // Read GPS location and send it over MQTT
    float lat=0, lon=0, speed=0, alt=0, accuracy=0;
    int   vsat=0, usat=0, year=0, month=0, day=0, hour=0, min=0, sec=0;
    
    for (int8_t i = 15; i; i--) {
      Serial.println("Requesting current GPS/GNSS/GLONASS location");
      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
          &year, &month, &day, &hour, &min, &sec)) {
        
        // Send over MQTT
        sendLocation(lat, lon, alt, speed, accuracy);
        break;
      } 
      else {
        Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
        delay(15000L);  // Wait 15s and try again
      }
    }
  }

  // === Check inactivity ===
  if (millis() - lastMotion > motionTimeout) {
    Serial.println("🛑 No motion – going to sleep...");

    // Battery status before sleep
    sendBatteryStatus();

    modem.gprsDisconnect();
    GPSTurnOff();
    modemPowerOff();

    // Prepare for wake on motion (SW-420 sensor, etc.)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1); // modify pin as needed
    delay(100);
    esp_deep_sleep_start();
  }

  mqttClient.loop();
}

// // ==== MPU Initialization ====
// void initMPU() {
//   mpu.initialize();
//   mpu.setMotionDetectionThreshold(2);  // ευαισθησία
//   mpu.setMotionDetectionDuration(5);   // σταθερή επιτάχυνση
//   mpu.setInterruptMode(1);
//   mpu.setDHPFMode(1); // High-pass filter
//   mpu.setIntMotionEnabled(true);
//   delay(100);
// }


float ReadBatteryVoltage() {
    analogSetAttenuation(ADC_ATTEN);
    analogReadResolution(ADC_RES);

    uint32_t raw_mv = analogReadMilliVolts(BOARD_BAT_ADC_PIN);  // mV
    float voltage = (raw_mv / 1000.0) * VOLTAGE_DIVIDER; // convert to volts

    // sanity check
    if (voltage < 2.5 || voltage > 5.0) return 0;
    return voltage;
}

// ---- Convert voltage to percentage ----
int BatteryPercent(float voltage) {
    if (voltage <= 3.0) return 0;
    if (voltage >= 4.2) return 100;
    return (int)(((voltage - 3.0) / (4.2 - 3.0)) * 100);
}