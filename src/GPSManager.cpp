#include <Arduino.h>
#include "utilities.h"
#include "GPSManager.h"
#include "modemManager.h"
#include <TinyGsmClient.h>

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

void sendCoords(float lat, float lon) {

//   int err = http.post("/?id="+myid+"&lat="+lat+"&lon="+lon+""); 
//    if (err != 0)
//   {
//     SerialMon.println(F("failed to connect"));
//     delay(10000);
//     return;
//   }

//   int status = http.responseStatusCode();

//   if (!status)
//   {
//     delay(10000);
//     return;
//   }

//   String body = http.responseBody();
//   SerialMon.println(F("Response:"));
//   SerialMon.println(body);

//   // Shutdown
//   http.stop();
//   SerialMon.println(F("Server disconnected"));
}