#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "modemManager.h"
#include "mqttManager.h"
// #include "MPU6050.h"
#include <BLEDevice.h>
// #include <WiFi.h>
// #include <WiFiClient.h>


// // WiFiClient wifiClient;

// // PubSubClient mqttClient(wifiClient);

// ==== BLE Key Fob ====
BLEScan* pBLEScan;
bool keyFobFound = false;

// ==== Tracking ====
unsigned long sendInterval = 15000; // κάθε 15s
unsigned long lastSend = 0;
unsigned long lastMotion = 0;
const unsigned long motionTimeout = 60 * 1000UL; // 1 λεπτό

// === FUNCTIONS ===
void sleepNow();

bool firstBoot = true;
bool normalBoot = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESPTracer starting...");

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wakeup cause: %d\n", cause);

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Normal boot");
    normalBoot = true;

  } else {
    Serial.println("Wakeup from EXT0 (motion)");
  }

  // Pull down DTR to ensure the modem is not in sleep state
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  // Power ON sequence for SIM7000
  modemPowerOn();
  delay(5000); // Wait for modem to start


  Serial.println("Check modem online .");
  while (!modem.testAT()) {
    Serial.print("."); 
    delay(500);
  }
  Serial.println("Modem is online!");

  // Set LED OFF
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }

  delay(500);

  // Connect to network
  Serial.print("Trying to connect to APN: ");
  Serial.print(APN);
  while (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println("GPRS connect failed, retrying...");
    Serial.println("signal quality: " + String(modem.getSignalQuality()));
    checkModemStatus();
    delay(4000);
  }

  // Check GPRS connection
  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
    Serial.print("Local IP: ");
    Serial.println(modem.getLocalIP());
  } else {
    Serial.println("GPRS not connected");
  }
  
  // Enable GPS
  // GPSTurnOn();
  Serial.println("Enabling GPS/GNSS/GLONASS");
  while (!modem.enableGPS()) {
      Serial.print(".");
  }
  Serial.println();
  Serial.println("GPS Enabled");
  delay(500);

  // Connect MQTT
  connectToMQTT();
  delay(500);

  // === BLE key fob scan - Every time ESP wakes up ===
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);  // Set scan interval to 100ms
  pBLEScan->setWindow(99);   // Set scan window to 99ms (less or equal to setInterval value)

  BLEScanResults results = pBLEScan->start(3, false);
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
      // Don't need all this data yet
      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
          &year, &month, &day, &hour, &min, &sec)) {
        
        // Send over MQTT
        sendLocation(lat, lon, alt, speed, accuracy);
        break;
      } 
      else {
        Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
        delay(200); // wait before retry
        // Trying to optimize battery life - better than delaying
        esp_sleep_enable_timer_wakeup(15 * 1000000ULL);  // sleep for 15 sec and try again
        esp_light_sleep_start();
      }
    }
  }

  // === Check inactivity ===
  if (millis() - lastMotion > motionTimeout) {
    Serial.println("🛑 No motion for " + String(motionTimeout / 1000) + " seconds.");
    sleepNow();
  }

  if (normalBoot) {
    sleepNow();
  }

  mqttClient.loop();
}

void sleepNow () {
  // Battery status before sleep
  sendBatteryStatus();

  // Shutdown modem and GPS to save power
  modem.gprsDisconnect();
  // GPSTurnOff();
  modem.disableGPS();


  Serial.println("Enter modem power off!");

  if (modem.poweroff()) {
      Serial.println("Modem enter power off modem!");
  } else {
      Serial.println("modem power off failed!");
  }

  delay(5000);

  Serial.println("Check modem response .");
  while (modem.testAT()) {
      Serial.print("."); delay(500);
  }
  Serial.println("Modem is not response ,modem has sleep!");

  delay(5000);

  // Prepare for wake on motion (SW-420 sensor, etc.)
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  // pinMode(GPIO_NUM_32, INPUT);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1);
  SerialAT.end();
  btStop(); // Stop Bluetooth to save power
  delay(200);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}