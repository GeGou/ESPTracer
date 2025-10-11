#pragma once

// WiFi Credentials
#define WIFI_SSID "your_wifi_ssid" // Replace with your WiFi SSID
#define WIFI_PASSWORD "your_wifi_password" // Replace with your WiFi password


// MQTT Broker Configuration
#define MQTT_BROKER "mqtt_ip" // Replace with your MQTT broker IP or hostname
#define MQTT_PORT port_number   // Replace with your MQTT port number
#define MQTT_USERNAME "mqtt_username"   // Replace with your MQTT username
#define MQTT_PASSWORD "mqtt_password"   // Replace with your MQTT password
#define MQTT_TOPIC_LOC "vehicle/location"
#define MQTT_TOPIC_TAG "vehicle/keyfob"

// GPRS Settings
#define GPRS_USER ""    // GPRS username, if required
#define GPRS_PASS ""    // GPRS password, if required
#define APN "apn"   // Access Point Name for GPRS connection
#define GSM_PIN "" // SIM card PIN (if any)

#define ITAG_MAC_ADDRESS "ff:ff:ff:ff:ff:ff" // iTag's MAC address, replace with your iTag's actual MAC address