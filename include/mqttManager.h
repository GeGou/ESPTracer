#pragma once
#include <Arduino.h>

void callback(char* topic, byte* payload, unsigned int length);
void connectToMQTT();
void sendLocation(float lat, float lng, float alt=0, float speed=0, float accuracy=0);
void sendKeyFobStatus(bool found);
void sendBatteryStatus();
void sendModemStatus();

// Read battery voltage
float ReadBatteryVoltage();

// Convert voltage to percentage
int BatteryPercent(float voltage);