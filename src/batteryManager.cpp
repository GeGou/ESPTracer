#include <Arduino.h>
#include "batteryManager.h"
#include "utilities.h"

float ReadBatteryVoltage() {
    analogSetAttenuation(ADC_ATTEN);
    analogReadResolution(ADC_RES);

    uint32_t raw_mv = analogReadMilliVolts(BOARD_BAT_ADC_PIN);  // mV
    float voltage = (raw_mv / 1000.0) * VOLTAGE_DIVIDER; // convert to volts

    // sanity check
    if (voltage < 2.5 || voltage > 5.0) return 0;
    return voltage;
}

// ---- Convert voltage to percentage ----
int BatteryPercent(float voltage) {
    if (voltage <= 3.0) return 0;
    if (voltage >= 4.2) return 100;
    return (int)(((voltage - 3.0) / (4.2 - 3.0)) * 100);
}