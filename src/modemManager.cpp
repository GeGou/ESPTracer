#include "modemManager.h"
#include "utilities.h"

void modemPowerOn() {
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(1500);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(1000);
}

void modemPowerOff(){
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(1500);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(1000);
}

void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void checkModemStatus() {
  Serial.println("Checking modem status...");
  Serial.print("SIM status: ");
  Serial.println(modem.getSimStatus());
  Serial.print("Network registration: ");
  Serial.println(modem.isNetworkConnected() ? "Connected" : "Not connected");
  Serial.print("Signal quality: ");
  Serial.println(modem.getSignalQuality());
}

void GPSTurnOn(void) {
  Serial.println("Start positioning . Make sure to locate outdoors.");
  // Enable the power to GPS
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    Serial.println(" SGPIO=0,4,1,1 false ");
  }
  // Enable GPS
  modem.enableGPS();
}

void GPSTurnOff(void) {
  // Disable GPS
  modem.disableGPS();
  // Disable the power to GPS
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    Serial.println(" SGPIO=0,4,1,0 false ");
  }
}