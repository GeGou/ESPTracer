#include <Arduino.h>
#include <mqttManager.h>
#include "config.h"
#include "utilities.h"

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

PubSubClient mqttClient(client);

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // Serial.print("MQTT Message [");
  // Serial.print(topic);
  // Serial.print("]: ");
  // Serial.println(message);

  // Check for reboot command
  if (String(topic) == MQTT_TOPIC_COMMAND) {
    message.trim();
    if (message.equalsIgnoreCase("reboot")) {
      Serial.println("Reboot command received via MQTT!");
      delay(500);
      ESP.restart();
    }
  }
}

void connectToMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(callback);

  while (!mqttClient.connected()) {
    
    Serial.println("Connection to MQTT Broker ...");
    if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(MQTT_TOPIC_KEYFOB);
      mqttClient.subscribe(MQTT_TOPIC_LOC);
      mqttClient.subscribe(MQTT_TOPIC_BAT);
      mqttClient.subscribe(MQTT_TOPIC_MODEM);
      mqttClient.subscribe(MQTT_TOPIC_STATUS);
      mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    } else {
      Serial.print("Failed to connect to MQTT broker. Error: ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void sendLocation(float lat, float lng, float alt, float speed, float accuracy) {
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

// Στέλνει την κατάσταση λειτουργίας της συσκευής (sleeping: true/false) στο MQTT_TOPIC_STATUS.
// retain=true ώστε όποιος συνδεθεί στο topic να βλέπει αμέσως την τελευταία γνωστή κατάσταση.

// FIX: προστέθηκε mqttClient.loop() + delay μετά το publish, γιατί σε cellular
// σύνδεση (μέσω modem/AT) η μεταφορά του πακέτου έχει μεγαλύτερο latency απ' ό,τι
// σε WiFi - χωρίς αυτό, το sleepNow() προχωρούσε αμέσως σε gprsDisconnect() πριν
// προλάβει να ολοκληρωθεί πραγματικά η αποστολή του "sleeping" μηνύματος.
void sendDeviceStatus(bool sleeping) {
  String payload = "{\"sleeping\": " + String(sleeping ? "true" : "false") + "}";
  bool ok = mqttClient.publish(MQTT_TOPIC_STATUS, payload.c_str(), true); // retained
  Serial.println("🟢 Device status: " + String(sleeping ? "sleeping" : "awake") + (ok ? "" : " (publish failed)"));
 
  mqttClient.loop();
  delay(1000); // δίνουμε χρόνο στο modem/cellular link να ολοκληρώσει την αποστολή
  mqttClient.loop();
}

void sendModemStatus() {
  String modemInfo = modem.getModemInfo();
  String signalQuality = String(modem.getSignalQuality());

  String payload = "{\"modemInfo\": \"" + modemInfo + "\", \"signalQuality\": " + signalQuality + "}";
  mqttClient.publish(MQTT_TOPIC_MODEM, payload.c_str());
  Serial.println("Modem Info: " + modemInfo);    
  Serial.println("Signal Quality: " + signalQuality);    
}

// Read battery voltage
float ReadBatteryVoltage() {
  analogSetAttenuation(ADC_ATTEN);
  analogReadResolution(ADC_RES);

  uint32_t raw_mv = analogReadMilliVolts(BOARD_BAT_ADC_PIN);  // mV
  float voltage = (raw_mv / 1000.0) * VOLTAGE_DIVIDER; // convert to volts
  // uint32_t voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);  // mV
  // voltage *= VOLTAGE_DIVIDER; // convert to volts

  // sanity check
  if (voltage < 2.5 || voltage > 5.0) return 0;
  return voltage;
}

// Convert voltage to percentage
int BatteryPercent(float voltage) {
  if (voltage <= 3.4) return 0;
  if (voltage >= 4.2) return 100;
  return (int)(((voltage - 3.4) / (4.2 - 3.4)) * 100);
}