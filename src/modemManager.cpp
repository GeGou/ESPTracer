#include "modemManager.h"
#include "utilities.h"

void modemPowerOn() {
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000); // Delay time is critical for proper power on , 1000 ms
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
}

void modemPowerOff() {
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);    // hold low
  delay(1500);                            // 1.5 sec pulse
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);   // release
  delay(1000); // Wait for modem to power off

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
  modem.sendAT("+SGPIO=0,4,1,1"); //modem.sendAT("+CGPIO=0,48,1,1");
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
  modem.sendAT("+SGPIO=0,4,1,0"); //modem.sendAT("+CGPIO=0,48,1,0");
  if (modem.waitResponse(10000L) != 1) {
    Serial.println(" SGPIO=0,4,1,0 false ");
  }
}