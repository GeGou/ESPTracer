#include <Arduino.h>
#include <mqttManager.h>
#include "config.h"
#include "utilities.h"
#include "state.h"

// retain=true is used for topics that represent the current state of the device 
//(e.g., location, key fob status, battery status, device status, modem status).

// Added void mqttFlush() after publish, because on a cellular connection (via modem/AT),
// packet transmission has higher latency than on Wi-Fi—without this, sleepNow() would immediately
// proceed to gprsDisconnect() before the message had actually finished sending.

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

  // Check for reboot/power off/stop alarm command
  if (String(topic) == MQTT_TOPIC_COMMAND) {
    message.trim();
    if (message.equalsIgnoreCase("reboot")) {
      Serial.println("Reboot command received via MQTT.");
      mqttFlush();
      ESP.restart();
    }
    else if (message.equalsIgnoreCase("power_off")) {
      Serial.println("Power off command received via MQTT.");
      mqttFlush();
      powerOffRequested = true;
    }
    else if (message.equalsIgnoreCase("stop_alarm")) {
      Serial.println("Stop alarm command received via MQTT.");
      mqttFlush();
      stopAlarmRequested = true;
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
      mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    } else {
      Serial.print("⚠️ Failed to connect to MQTT broker. Error: ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void publishLocation(float lat, float lng, float alt, float speed, float accuracy) {
  String payload = "{\"latitude\":" + String(lat, 6) + ",\"longitude\":" + String(lng, 6) + ",\"altitude\":" + 
    String(alt, 2) + ",\"speed\":" + String(speed, 2) + ",\"gps_accuracy\":" + String(accuracy, 2) + "}";
  if (!mqttClient.publish(MQTT_TOPIC_LOC, payload.c_str(), true)) { // retained
    Serial.println("⚠️  Failed to publish location data!");
  } else {
    Serial.println("📡 Sent: " + payload);
  }
}

// Do not need to use cellular connection to publish key fob status, because the BLE key fob is only detected when the device is awake (during ALARM).
void publishKeyFobStatus(bool found) {
  // String payload = "{\"status\":\"" + String(found ? "found" : "not_found") + "\"}";
  // if (!mqttClient.publish(MQTT_TOPIC_KEYFOB, payload.c_str(), true)) { // retained
  //   Serial.println("⚠️  Failed to publish key fob status!");
  // } else {
  //   if (found) {
  //     Serial.println("🔵 BLE key fob: found");
  //   } else {
  //     Serial.println("🔴 BLE key fob: not found");
  //   }
  // }
  if (found) {
    Serial.println("🔵 BLE key fob: found");
  } else {
    Serial.println("🔴 BLE key fob: not found");
  }
}

void publishBatteryStatus() {
  float voltage = ReadBatteryVoltage();
  int percent = BatteryPercent(voltage);

  String payload = "{\"battery\": " + String(voltage, 2) + ", \"batteryLevel\": " + String(percent) + "}";
  if (!mqttClient.publish(MQTT_TOPIC_BAT, payload.c_str(), true)) { // retained
    Serial.println("⚠️  Failed to publish battery status!");
  } else {
    Serial.println("Battery voltage: " + String(voltage, 2) + " V (" + String(percent) + "%)");    
  }
}

void publishDeviceStatus(bool sleeping) {
  String payload = "{\"sleeping\": " + String(sleeping ? "true" : "false") + "}";
  if (!mqttClient.publish(MQTT_TOPIC_STATUS, payload.c_str(), true)) { // retained
    Serial.println("⚠️  Failed to publish device status!");
  } else {
    if (sleeping) {
      Serial.println("🔵 Device status: sleeping");
    } else {
      Serial.println("🟢 Device status: awake");
    }
  }

  mqttFlush(); // give time for the message to be sent before sleeping
}

void publishModemStatus() {
  String modemInfo = modem.getModemInfo();
  String signalQuality = String(modem.getSignalQuality());

  String payload = "{\"modemInfo\": \"" + modemInfo + "\", \"signalQuality\": " + signalQuality + "}";
  if (!mqttClient.publish(MQTT_TOPIC_MODEM, payload.c_str(), true)) { // retained
    Serial.println("⚠️  Failed to publish modem status!");
  } else {
    Serial.println("Modem Info: " + modemInfo);    
    Serial.println("Signal Quality: " + signalQuality);    
  }
}

void publishAlarmEvent() {
  String payload = "{\"event\":\"ALARM_TRIGGERED\",\"uptime_ms\":" + String(millis()) + "}";
  if (!mqttClient.publish(MQTT_TOPIC_ALARM, payload.c_str(), false)) { // Not retained, because we want to notify only when the event occurs
    Serial.println("⚠️  Failed to publish alarm event!");
  } else {
    mqttFlush();
    Serial.println("🚨 Alarm event δημοσιεύτηκε: " + String(MQTT_TOPIC_ALARM));
  }
}
 
void publishStateTopic() {
  if (!mqttClient.publish(MQTT_TOPIC_STATE, stateNameOf(rtcDeviceState), true)) { // retained
    Serial.println("⚠️  Failed to publish state topic!");
  } else {
    mqttFlush();
    Serial.println("State topic ενημερώθηκε: " + String(MQTT_TOPIC_STATE) + " = " + String(stateNameOf(rtcDeviceState)));
  }
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

// Flush MQTT messages for a specified duration (default 500 ms)
void mqttFlush(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
      mqttClient.loop();
      delay(10);
  }
}