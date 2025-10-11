#include "modemManager.h"
#include "utilities.h"

void modemPowerOn(){
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
}

void modemPowerOff(){
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(1500);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
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


// // Helper: power on modem (simple PWRKEY pulse)
// inline void modemPowerOn() {
//     pinMode(MODEM_PWRKEY_PIN, OUTPUT);
//     digitalWrite(MODEM_PWRKEY_PIN, LOW);
//     delay(100);
//     digitalWrite(MODEM_PWRKEY_PIN, HIGH);
//     delay(1000);
//     digitalWrite(MODEM_PWRKEY_PIN, LOW);
// }