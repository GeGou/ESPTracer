// Arduino-based GPS + BLE Fob + Motion + MQTT tracker
// Modules: ESP32, SIM800L (GSM), MPU6050, BLE (for key fob detection)

#include <Arduino.h>
#include "config_private.h"
#include "utilities.h"

// #include "MPU6050.h"
#include <TinyGPSPlus.h>
// #define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#include <TinyGsmClient.h>
#include <BLEDevice.h>
#include <PubSubClient.h>

// #include <esp_sleep.h>


TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
TinyGPSPlus gps;

// BLE Key Fob
BLEScan* pBLEScan;
bool tagFound = false;

// ==== MPU6050 ====
// MPU6050 mpu;
// #define MPU_INT_PIN 34  // Πρέπει να είναι RTC_GPIO για wakeup
// volatile bool motionDetected = false;

// #define WAKEUP_PIN 2

// ==== Tracking ====
unsigned long sendInterval = 10000; // κάθε 10s
unsigned long lastSend = 0;
unsigned long lastMotion = 0;
const unsigned long motionTimeout = 10 * 60 * 1000UL; // 10 λεπτά

// ------------------------------------------------------

bool connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  Serial.print("Connecting to MQTT...");
  for (int i = 0; i < 5; i++) {
    if (mqtt.connect("ESP32Tracker")) {
      Serial.println("connected!");
      return true;
    }
    delay(2000);
  }
  Serial.println("MQTT connection failed");
  return false;
}

void sendLocation(float lat, float lng) {
  String payload = "{\"lat\":" + String(lat, 6) + ",\"lng\":" + String(lng, 6) + "}";
  mqtt.publish(MQTT_TOPIC_LOC, payload.c_str());
  Serial.println("📡 Sent: " + payload);
}

void sendTagStatus(bool found) {
  String payload = "{\"status\":\"" + String(found ? "found" : "not_found") + "\"}";
  mqtt.publish(MQTT_TOPIC_TAG, payload.c_str());
  Serial.println("🔵 BLE tag: " + String(found ? "found" : "not_found"));
}

// ------------------------------------------------------


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("🚗 ESP32 Crash Wake Tracker starting...");

  // Αν ΞΥΠΝΗΣΕ από motion
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("💥 Motion detected – wakeup event!");
  } else {
    Serial.println("Normal boot");
  }

  // Init modem
  setupModemSerial();
  powerOnModem();
  enableGPS(true);
  delay(3000);

  
  // Init BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);

  // Connect GPRS
  Serial.println("Connecting to network...");
  modem.restart();
  if (!modem.gprsConnect(APN, "", "")) {
    Serial.println("GPRS connect failed");
  }

  // Connect MQTT
  connectMQTT();

  lastMotion = millis();


  // Set LED OFF
  // pinMode(BOARD_LED_PIN, OUTPUT);
  // digitalWrite(BOARD_LED_PIN, HIGH);
  
  // Print modem info
  String modemName = modem.getModemName();
  delay(500);
  Serial.println("Modem Name: " + modemName);

  String modemInfo = modem.getModemInfo();
  delay(500);
  Serial.println("Modem Info: " + modemInfo);
}


  // BLEDevice::init("");
  // pBLEScan = BLEDevice::getScan();
  // pBLEScan->setActiveScan(true);
  // pBLEScan->setInterval(100);
  // pBLEScan->setWindow(99);

  // Ρυθμίζουμε wakeup από motion interrupt pin
  // esp_deep_sleep_enable_gpio_wakeup(1 << WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1); // HIGH = wake
  // Serial.println("💤 Πηγαίνει σε Deep Sleep...");
  // delay(100);
  // esp_deep_sleep_start();
// }

void loop() {
  // === BLE tag scan ===
  BLEScanResults results = pBLEScan->start(3, false);
  tagFound = false;
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice d = results.getDevice(i);
    if (d.getAddress().toString() == ITAG_MAC_ADDRESS) {
      tagFound = true;
    }
  }
  sendTagStatus(tagFound);

  // === GPS ===
  while (SerialAT.available()) gps.encode(SerialAT.read());
  if (gps.location.isUpdated() && millis() - lastSend > sendInterval) {
    lastSend = millis();
    sendLocation(gps.location.lat(), gps.location.lng());
  }

  // === Check inactivity ===
  if (millis() - lastMotion > motionTimeout) {
    Serial.println("🛑 No motion – going to sleep...");

    modem.gprsDisconnect();
    enableGPS(false);

    // Prepare for wake on crash (motion sensor)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1);
    delay(100);
    esp_deep_sleep_start();
  }

  mqtt.loop();
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

// // ==== GPRS + TCP Connection ====
// void initGPRS() {
//   sendAT("AT");
//   sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
//   sendAT("AT+SAPBR=3,1,\"APN\",\"" + String(apn) + "\"");
//   sendAT("AT+SAPBR=1,1");
//   delay(2000);
//   sendAT("AT+SAPBR=2,1");
// }

// void connectTCP() {
//   sendAT("AT+CIPSHUT");
//   sendAT("AT+CIPMUX=0");
//   sendAT("AT+CSTT=\"" + String(apn) + "\"");
//   sendAT("AT+CIICR");
//   delay(2000);
//   sendAT("AT+CIFSR");
//   sendAT("AT+CIPSTART=\"TCP\",\"" + String(mqtt_host) + "\"," + String(mqtt_port));
//   delay(2000);
//   sendMQTTConnect();
// }

// void shutdownGPRS() {
//   sendAT("AT+CIPSHUT");
//   sendAT("AT+SAPBR=0,1");
// }
