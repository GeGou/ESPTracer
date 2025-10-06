// ===========================================================
// utilities.h – Custom definitions for TTGO T-SIM7000G
// ===========================================================

#pragma once
#include <Arduino.h>

// --- Board identification ---
#define PRODUCT_MODEL_NAME "LilyGo T-SIM7000G"
#define TINY_GSM_MODEM_SIM7000SSL   // TinyGSM driver

// --- Modem pins ---
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
#define MOTION_INT_PIN   34   // συνδέεις το INT από MPU6050 ή ADXL345 εδώ

// --- SD card SPI ---
#define BOARD_MISO_PIN   2
#define BOARD_MOSI_PIN   15
#define BOARD_SCK_PIN    14
#define BOARD_SD_CS_PIN  13

// --- Battery / Solar monitoring ---
#define BOARD_BAT_ADC_PIN   35
#define BOARD_SOLAR_ADC_PIN 36

// --- Serial for modem ---
#define SerialAT Serial1

// Helper: start the modem UART
inline void setupModemSerial(uint32_t baud = 115200) {
    SerialAT.begin(baud, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
}

// Helper: power on modem (simple PWRKEY pulse)
inline void powerOnModem() {
    pinMode(MODEM_PWRKEY_PIN, OUTPUT);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(MODEM_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
}

// Optional: enable GPS
inline void enableGPS(bool on = true) {
    pinMode(MODEM_GPS_EN_PIN, OUTPUT);
    digitalWrite(MODEM_GPS_EN_PIN, on ? MODEM_GPS_EN_LEVEL : !MODEM_GPS_EN_LEVEL);
}