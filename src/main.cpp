// Arduino-based GPS + BLE Fob + Motion + MQTT tracker
// Modules: ESP32, SIM800L (GSM), MPU6050, BLE (for key fob detection)

#include <Arduino.h>
#include "config_private.h"
#include "utilities.h"
#include "GPSManager.h"
#include "modemManager.h"
// #include "MPU6050.h"
#include <BLEDevice.h>


TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);
TinyGPSPlus gps;

// BLE Key Fob
BLEScan* pBLEScan;
bool tagFound = false;

// ==== MPU6050 ====
// MPU6050 mpu;
// #define MPU_INT_PIN 34  // Πρέπει να είναι RTC_GPIO για wakeup
// volatile bool motionDetected = false;


// ==== Tracking ====
unsigned long sendInterval = 10000; // κάθε 10s
unsigned long lastSend = 0;
unsigned long lastMotion = 0;
const unsigned long motionTimeout = 60 * 1000UL; // 1 λεπτό

// ------------------------------------------------------

// bool connectMQTT() {
//   mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
//   Serial.print("Connecting to MQTT...");
//   for (int i = 0; i < 5; i++) {
//     if (mqttClient.connect("ESP32Tracker")) {
//       Serial.println("connected!");
//       return true;
//     }
//     delay(2000);
//   }
//   Serial.println("MQTT connection failed");
//   return false;
// }

void connectToMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    // mqttClient.setCallback(callback);
  
    while (!mqttClient.connected()) {
      
        Serial.println("Connection to MQTT Broker ...");
        if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("Connected to MQTT broker");
            // mqttClient.subscribe(MQTT_TOPIC_GPS);  // Subscribe to topic
            mqttClient.subscribe(MQTT_TOPIC_TAG);
            mqttClient.subscribe(MQTT_TOPIC_LOC);
        } else {
            Serial.print("Failed to connect to MQTT broker. Error: ");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

void sendLocation(float lat, float lng) {
  String payload = "{\"lat\":" + String(lat, 6) + ",\"lng\":" + String(lng, 6) + "}";
  mqttClient.publish(MQTT_TOPIC_LOC, payload.c_str());
  Serial.println("📡 Sent: " + payload);
}

void sendTagStatus(bool found) {
  String payload = "{\"status\":\"" + String(found ? "found" : "not_found") + "\"}";
  mqttClient.publish(MQTT_TOPIC_TAG, payload.c_str());
  Serial.println("🔵 BLE tag: " + String(found ? "found" : "not_found"));
}

// ------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("🚗 ESP32 Crash Wake Tracker starting...");

  // Αν ΞΥΠΝΗΣΕ από motion
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("💥 Motion detected – wakeup event!");
  } else {
    Serial.println("Normal boot");
  }

  // Set LED OFF
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // setupModemSerial();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  delay(10000); // Wait for serial to initialize
  // Init modem
  modemPowerOn();

  Serial.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }

  delay(1000);

  
  // Init BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  // pBLEScan->setInterval(100);
  // pBLEScan->setWindow(99);

  // Print modem info
  // String modemName = modem.getModemName();
  // delay(500);
  // Serial.println("Modem Name: " + modemName);

  // String modemInfo = modem.getModemInfo();
  // delay(500);
  // Serial.println("Modem Info: " + modemInfo);

  // Connect to network
  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.print(APN);
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println(" fail");
    checkModemStatus();
    // Serial.println("signal quality: " + String(modem.getSignalQuality()));
    delay(10000);
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
  enableGPS();

  // Connect MQTT
  connectToMQTT();

  lastMotion = millis();
}

void loop() {
  // === BLE tag scan ===
  BLEScanResults results = pBLEScan->start(4, false);
  tagFound = false;
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.getAddress().toString() == ITAG_MAC_ADDRESS) {
      tagFound = true;
    }
  }
  sendTagStatus(tagFound);

  // === GPS ===
  while (SerialAT.available()) {
    gps.encode(SerialAT.read());
  }
  if (gps.location.isUpdated() && millis() - lastSend > sendInterval) {
    lastSend = millis();
    sendLocation(gps.location.lat(), gps.location.lng());
  }

  // === Check inactivity ===
  if (millis() - lastMotion > motionTimeout) {
    Serial.println("🛑 No motion – going to sleep...");

    modem.gprsDisconnect();
    disableGPS();

    // Prepare for wake on crash (motion sensor)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1);
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