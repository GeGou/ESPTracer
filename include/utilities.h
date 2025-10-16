// ===========================================================
// utilities.h – Custom definitions for TTGO T-SIM7000G
// ===========================================================

#pragma once
#include <Arduino.h>

// --- Board identification ---
#define PRODUCT_MODEL_NAME "LilyGo T-SIM7000G"
#define TINY_GSM_MODEM_SIM7000 // Modem is SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1K

// --- Serial for modem ---
#define SerialAT Serial1

#include <TinyGsmClient.h>
#include <PubSubClient.h>

extern TinyGsm modem;
extern TinyGsmClient client;
extern PubSubClient mqttClient;

// --- Modem pins ---
#define UART_BAUD        115200
#define MODEM_TX_PIN     27
#define MODEM_RX_PIN     26
#define MODEM_PWRKEY_PIN 4
#define MODEM_DTR_PIN    25

#define MODEM_GPS_EN_PIN 48
#define MODEM_GPS_EN_LEVEL 1

// --- LED ---
#define BOARD_LED_PIN    12
#define LED_ON           LOW

// --- Motion sensor pin (Wake pin) ---
#define MOTION_INT_PIN   32   // συνδέεις το INT από MPU6050 ή ADXL345 εδώ

// --- SD card SPI ---
#define BOARD_MISO_PIN   2
#define BOARD_MOSI_PIN   15
#define BOARD_SCK_PIN    14
#define BOARD_SD_CS_PIN  13

// --- Battery / Solar monitoring ---
#define BOARD_BAT_ADC_PIN   35
#define BOARD_SOLAR_ADC_PIN 36
#define ADC_ATTEN ADC_11db  // ADC attenuation
#define VOLTAGE_DIVIDER 2.0 // Divider R1=R2=100k
#define ADC_RES 12          // ADC resolution