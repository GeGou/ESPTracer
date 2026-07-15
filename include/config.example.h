#pragma once

// WiFi Credentials
#define WIFI_SSID "your_wifi_ssid" // Replace with your WiFi SSID
#define WIFI_PASSWORD "your_wifi_password" // Replace with your WiFi password

// MQTT Broker Configuration
#define MQTT_BROKER "mqtt_ip" // Replace with your MQTT broker IP or hostname
#define MQTT_PORT port_number   // Replace with your MQTT port number
#define MQTT_USERNAME "mqtt_username"   // Replace with your MQTT username
#define MQTT_PASSWORD "mqtt_password"   // Replace with your MQTT password
#define MQTT_TOPIC_LOC "esptracer/location"
#define MQTT_TOPIC_KEYFOB "esptracer/keyfob"
#define MQTT_TOPIC_BAT "esptracer/battery"
#define MQTT_TOPIC_MODEM "esptracer/modem"
#define MQTT_TOPIC_COMMAND "esptracer/command"
#define MQTT_TOPIC_ALARM "esptracer/alarm" // Not retained -- immediate notification of ALARM event
#define MQTT_TOPIC_STATE "esptracer/state" // Retained -- current state (DISARMED/ARMED/ALARM)


// GPRS Settings
#define GPRS_USER ""    // GPRS username, if required
#define GPRS_PASS ""    // GPRS password, if required
#define APN "apn"   // Access Point Name for GPRS connection
#define GSM_PIN "" // SIM card PIN (if any)

#define KEYFOB_MAC_ADDRESS "ff:ff:ff:ff:ff:ff" // iTag's MAC address, replace with your iTag's actual MAC address

// state machine thresholds
#define MOTION_TIMEOUT_MS (2UL * 60UL * 1000UL) // 2 minutes of inactivity -> end of ALARM tracking 
#define ALARM_SEND_INTERVAL_MS 5000    // GPS/MQTT interval DURING ALARM
#define BLE_RESCAN_INTERVAL_MS 30000    // rescan keyfob during ALARM
#define MOTION_WINDOW_MS 5000     // consensus window for motion filter
#define MOTION_CONSENSUS_COUNT 3    // minimum events within the window