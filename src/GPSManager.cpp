#include <Arduino.h>
#include "utilities.h"
#include "GPSManager.h"
#include "modemManager.h"
#define TINY_GSM_DEBUG Serial
#include <TinyGsmClient.h>
#include <HttpClient.h>

// TinyGsm modem(SerialAT);
// TinyGsmClient client(modem);
// HttpClient http(client, server, port);

void enableGPS(void) {
  Serial.println("Start positioning . Make sure to locate outdoors.");
  Serial.println("The blue indicator light flashes to indicate positioning.");
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
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