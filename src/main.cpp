#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "modemManager.h"
#include "mqttManager.h"
#include <BLEDevice.h>
#include "state.h"
// #include <WiFi.h>
// #include <WiFiClient.h>

// ==================================================================
// SENSOR SELECTION — Enable whatever you want. In the future, you may have
// both enabled at the same time (e.g., OR logic on the interrupt) if
// you decide to combine them—for now, if you enable
// both, the WAKE_PIN below will temporarily point to SW420_PIN.
// ==================================================================
#define USE_SW420
// #define USE_MPU6050

#ifdef USE_MPU6050
#include <Wire.h> // MPU6050-specific: needs I2C
#endif

// === FUNCTIONS ===
void sleepNow();
void sleepSilent();
void scanForKeyFob();
void IRAM_ATTR onMotionISR();
void disarmedFlow();
void startAlarmTracking();
void fullPowerOff();

#ifdef USE_MPU6050
void writeMPU(uint8_t reg, uint8_t data); // MPU6050-specific (I2C)
uint8_t readMPU(uint8_t reg);              // MPU6050-specific (I2C)
void setupMPUMotionInterrupt();            // MPU6050-specific
#endif
#ifdef USE_SW420
void setupSW420MotionInterrupt();          // net digital, no I2C
#endif

// Which pin is ultimately used for `attachInterrupt` / `ext0 wakeup`,
// depending on which sensor is active.
#if defined(USE_SW420) && defined(USE_MPU6050)
  #define WAKE_PIN SW420_PIN //Both are active — temporarily prioritize SW420; see the "combine-both" comment for future reference
#elif defined(USE_SW420)
  #define WAKE_PIN SW420_PIN
#elif defined(USE_MPU6050)
  #define WAKE_PIN MOTION_INT_PIN
#else
  #error "You must enable at least one of the USE_SW420 / USE_MPU6050"
#endif


// ==================================================================
// STATE MACHINE (Monimoto-style): DISARMED / ARMED / ALARM
//   DISARMED : keyfob found -> one location report, sleep. No notification.
//   ARMED    : keyfob not found, FIRST time -> silent arming, NO modem,
//              goes directly to sleep, armed.
//   ALARM    : was already ARMED and motion event occurred -> escalation, full
//              tracking + immediate notification.
// The rtcDeviceState persists through deep sleep via RTC memory (RTC_DATA_ATTR).
// ==================================================================

RTC_DATA_ATTR int rtcDeviceState = STATE_DISARMED;
 
const char* stateNameOf(int s) {
  switch (s) {
    case STATE_DISARMED: return "DISARMED";
    case STATE_ARMED:    return "ARMED";
    case STATE_ALARM:    return "ALARM";
    default:             return "UNKNOWN";
  }
}

// ==== BLE Key Fob ====
BLEScan* pBLEScan;
bool keyFobFound = false;

// ==== Tracking ====
unsigned long lastSend = 0;
unsigned long lastBleScan = 0;
unsigned long lastMotion = 0;

// ==== Motion detection ====
// The ISR does not make I2C calls (safe within interrupt context).
// It simply sets a flag — the cleanup is done in the loop() (only for MPU6050, see there).
volatile unsigned long lastMotionISR = 0;
volatile bool motionFlag = false;

void IRAM_ATTR onMotionISR() {
  unsigned long now = millis();
  if (now - lastMotionISR > 1000) { // hardware debounce 1000ms, within the ISR
    motionFlag = true;
    lastMotionISR = now;
  }
}

// Software consensus motion filter (only while the ALARM tracking loop is active)
unsigned long motionWindowStart = 0;
int motionEventsInWindow = 0;

  // MQTT commands (μέσω MQTT_TOPIC_COMMAND) -- λειτουργούν ΜΟΝΟ όσο η
  // συσκευή είναι ήδη ξύπνια/συνδεδεμένη (δηλαδή κατά τη διάρκεια ALARM).
volatile bool stopAlarmRequested = false;
volatile bool powerOffRequested = false;

#ifdef USE_MPU6050
//////////////////////////////////////
// ---- MPU6050-specific I2C helpers ----
void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readMPU(uint8_t reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void setupMPUMotionInterrupt() {
  writeMPU(0x6B, 0x00);   // PWR_MGMT_1: exit sleep, internal 8MHz osc
  delay(10);

  writeMPU(0x6C, 0x07);   // PWR_MGMT_2: STBY_XG/YG/ZG=1 -> gyro OFF, accel only
  writeMPU(0x1C, 0x00);   // ACCEL_CONFIG: ±2g (maximum sensitivity)
  // writeMPU(0x1C, 0x08);  // AFS_SEL=1 -> ±4g, less sensitive to noise

  writeMPU(0x1F, 35);     // MOT_THR: motion threshold (~70mg). Tune it empirically.
  writeMPU(0x20, 30);     // MOT_DUR: motion duration ~30ms, filters isolated spikes

  writeMPU(0x69, 0x15);   // MOT_DETECT_CTRL

  writeMPU(0x37, 0x30);   // INT_PIN_CFG: active-high, push-pull, latch, clear-on-read

  writeMPU(0x6C, 0x87);   // STBY_XG/YG/ZG=1 (0x07) | LP_WAKE_CTRL=10 (20Hz) => 0x87
  writeMPU(0x6B, 0x20);   // PWR_MGMT_1: CYCLE=1
  delay(100);             // settle time before we enable the interrupt

  writeMPU(0x38, 0x40);   // INT_ENABLE: motion interrupt enabled (is armed last so it doesn't trigger on startup)

  readMPU(0x3A); // INT_STATUS (clear-on-read) - clears any transient interrupts

  pinMode(MOTION_INT_PIN, INPUT);
}
////////////////////////////////////////
#endif // USE_MPU6050

#ifdef USE_SW420
// ==================================================================
// SW-420 setup.
// Confirmation of polarity on the module via sw420_test.cpp:
// e.g., idle  = LOW
//       pulse = HIGH (very short, a few ms, on each vibration/beat)
// Therefore: RISING edge for the interrupt, and ext0 wake on HIGH (1).
// The sensitivity is adjusted ONLY with the potentiometer on the module.
// ==================================================================
void setupSW420MotionInterrupt() {
  // pinMode(SW420_PIN, INPUT); // #define SW420_PIN <GPIO> στο config.h
  pinMode(SW420_PIN, INPUT_PULLUP); // #define SW420_PIN <GPIO> στο config.h
}
#endif // USE_SW420

void scanForKeyFob() {
  keyFobFound = false;

  BLEScanResults results = pBLEScan->start(5, false);
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);

    if (device.getAddress().toString() == KEYFOB_MAC_ADDRESS) {
      Serial.println("Found key fob with RSSI: " + String(device.getRSSI()));
      if (device.getRSSI() > BLE_RSSI) {
        keyFobFound = true;
      }
    }
  }
  pBLEScan->clearResults(); // clear memory after each scan
  publishKeyFobStatus(keyFobFound);
}
 
// ==================================================================
// Common "power-up" sequence for modem/GPRS/GPS/MQTT -- used by both DISARMED
// (a single report) and ALARM (full tracking).
// ==================================================================
static void powerUpConnectivity() {
  // Pull down DTR to ensure the modem is not in sleep state
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);
 
  // Power ON sequence for SIM7000
  modemPowerOn();
  delay(5000);
 
  Serial.println("Check modem online.");
  int attempts = 0;
  bool modemOK = modem.testAT();
  while (!modemOK) {
    Serial.print(".");
    delay(500);
    attempts++;
 
    if (attempts % 10 == 0) {
      Serial.println("\nModem is not responding, trying modem restart!");
      modem.restart();
      delay(3000);  // Wait for modem to restart
    }
 
    if (attempts > 20) {
      Serial.println("Modem still not responding after restart, restarting ESP32!");
      ESP.restart();
    }
 
    modemOK = modem.testAT();
  }
  Serial.println("Modem is online!");
 
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
 
  delay(500);
 
  // Connect to network
  Serial.print("Trying to connect to APN: ");
  Serial.println(APN);
  while (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println("GPRS connect failed, retrying...");
    Serial.println("signal quality: " + String(modem.getSignalQuality()));
    checkModemStatus();
    delay(4000);
  }
 
  // Check GPRS connection
  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
    Serial.print("Local IP: ");
    Serial.println(modem.getLocalIP());
  } else {
    Serial.println("GPRS not connected");
  }
 
  // Enable GPS
  GPSTurnOn();
  delay(500);
 
  // Connect MQTT
  connectToMQTT();
  delay(500);

  // NEW: registration to receive commands (STOP_ALARM / POWER_OFF) -- works only
  // as long as the device remains awake/connected (i.e., during ALARM).
  mqttClient.setCallback(callback);
  mqttClient.subscribe(MQTT_TOPIC_COMMAND);
}
 
// ==================================================================
// DISARMED flow: a single location report before sleep. No repeated
// transmission, does not enter the loop().
// ==================================================================
void disarmedFlow() {
  powerUpConnectivity();
 
  publishDeviceStatus(false);
  publishBatteryStatus();
  delay(500);
  publishModemStatus();
  delay(500);
 
  publishStateTopic(); // updates the state topic when returning from ARMED/ALARM
 
  float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
  int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
 
  Serial.println("DISARMED: requesting one-shot location before sleep...");
  if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
    &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
    publishLocation(lat, lon, alt, speed, accuracy);
  } else {
    Serial.println("DISARMED: no GPS fix found, sleeping without location.");
  }
  delay(500);
 
  sleepNow();
}
 
// ==================================================================
// ALARM tracking: full power-up + IMMEDIATE alert before we even wait for a GPS fix
// ==================================================================
void startAlarmTracking() {
  powerUpConnectivity();
 
  publishStateTopic();
  publishAlarmEvent(); // FIRST, the notification, before we wait for a GPS fix
 
  publishDeviceStatus(false);
  publishBatteryStatus();
  delay(500);
  publishModemStatus();
  delay(500);
 
  lastSend = 0; // so that the loop() sends a GPS fix immediately on the first iteration
}
/////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  Serial.println("ESPTracer starting...");

  // Set LED OFF
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

#ifdef USE_MPU6050
  Wire.begin(21, 22); // SDA, SCL
  setupMPUMotionInterrupt();
#endif
#ifdef USE_SW420
  setupSW420MotionInterrupt();
#endif

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wakeup cause: %d\n", cause);

  bool normalBoot = false;
  if (cause != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Normal boot");
    normalBoot = true;
  } else {
    Serial.println("Wakeup from EXT0 (motion)");
  }

  // Enable the interrupt to detect motion while the device is awake
  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), onMotionISR, RISING);

  // ==================================================================
  // === BLE key fob scan — performed FIRST, before the modem/GPS powers on ===
  // The BLE scan is handled by the ESP32 (low power consumption), while the SIM7000 modem
  // costs much more. By checking the key fob first, we determine WHETHER
  // a full tracking session is even necessary, before wasting power.
  // ==================================================================
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(false); // Passive scan to save power
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  scanForKeyFob();
  lastBleScan = millis();
  bool keyfobPresent = keyFobFound;
  bool effectiveArmed = !keyfobPresent;

  DeviceState newState;
  if (!effectiveArmed) {
    newState = STATE_DISARMED;
  } else if (normalBoot) {
    // First boot/reset while armed -> we start fresh from ARMED, never ALARM.
    newState = STATE_ARMED;
  } else if (rtcDeviceState == STATE_ARMED || rtcDeviceState == STATE_ALARM) {
    // Already armed from before, NEW motion event -> escalation to ALARM
    // (2nd+ consecutive motion event while armed = real motion, not just
    // the owner moving away once armed).
    newState = STATE_ALARM;
  } else {
    // First time armed (just after the keyfob left) -> silent arming.
    newState = STATE_ARMED;
  }

  Serial.println("State: " + String(stateNameOf(rtcDeviceState)) + " -> " + String(stateNameOf(newState)));
  rtcDeviceState = newState;

  lastMotion = millis();
  motionWindowStart = millis();
  motionEventsInWindow = 0;

  if (newState == STATE_DISARMED) {
    disarmedFlow();
    return; // It never gets here -- disarmedFlow() calls sleepNow()
  }
 
  if (newState == STATE_ARMED) {
    Serial.println("ARMED (silent) -- no modem, sleeping without location.");
    sleepSilent();
    return; // It never gets here
  }
 
  // STATE_ALARM
  startAlarmTracking(); // It returns normally; the loop() function takes over tracking

}

void loop() {
  unsigned long now = millis();
  
  mqttClient.loop(); // It MUST be running so that the commands reach callback()

  // Check for commands received via MQTT_TOPIC_COMMAND
  if (powerOffRequested) {
    Serial.println("Command POWER_OFF received -- full, permanent shutdown.");
    fullPowerOff();
    return; // It never gets here
  }
 
  if (stopAlarmRequested) {
    Serial.println("Command STOP_ALARM received -- stopping the current alarm.");
    stopAlarmRequested = false;
    rtcDeviceState = STATE_ARMED; // remains "asleep", will wake up on new motion
    publishStateTopic();
    sleepNow();
    return; // It never gets here
  }

  // This loop() runs ONLY while rtcDeviceState == STATE_ALARM.
  // Update last motion time if motion detected — with software consensus filter
  if (motionFlag) {
    motionFlag = false;
#ifdef USE_MPU6050
    readMPU(0x3A); // INT_STATUS: clear-on-read -- MPU6050-specific
#endif
    // FIX: sliding logic -- reset ONLY if there is an actual gap (>MOTION_WINDOW_MS)
    // from the PREVIOUS event, not from the start of a fixed window. Thus, consecutive
    // events (e.g., 1/sec due to hardware debounce) never “lose” the count due to absolute
    // time—only an actual pause in motion triggers a reset.   
    if (now - motionWindowStart > MOTION_WINDOW_MS) {
      motionEventsInWindow = 1;
    } else {
      motionEventsInWindow++;
    }
    motionWindowStart = now;  // update on EVERY event, not just reset

    if (motionEventsInWindow >= MOTION_CONSENSUS_COUNT) {
      lastMotion = now; // "real" continuous motion confirmed
      Serial.println("Confirmed motion (consensus) - Timer reset.");
    } else {
      Serial.println("Motion event (" + String(motionEventsInWindow) + "/" +
                      String(MOTION_CONSENSUS_COUNT) + ") - waiting for consensus.");
    }
  }

  // === BLE rescan - periodic rescan while we remain awake ===
  if (now - lastBleScan >= BLE_RESCAN_INTERVAL_MS) {
    lastBleScan = now;
    Serial.println("Re-scanning for key fob...");
    scanForKeyFob();
    // Note: In the future, we could switch from FULL to LIGHT here
    // if the key fob is detected again during a FULL tracking session.
    if (keyFobFound) {
      Serial.println("🔑 Keyfob returned during ALARM -> DISARMED, termination.");
      rtcDeviceState = STATE_DISARMED;
      publishStateTopic();
      sleepNow();
      return;
    }
  }

  // === GPS === (only in FULL mode, since the loop() runs only then)
  if (now - lastSend >= ALARM_SEND_INTERVAL_MS) {
    lastSend = now;

    float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
    int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

    Serial.println("Requesting current location");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
      &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      publishLocation(lat, lon, alt, speed, accuracy);
    } else {
      Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in " + String(ALARM_SEND_INTERVAL_MS / 1000) + "s.");
    }
  }

  // === Check inactivity ===
  if (now - lastMotion > MOTION_TIMEOUT_MS) {
    Serial.println("Stop - No motion for " + String(MOTION_TIMEOUT_MS / 1000) + " seconds.");
    // Remains "asleep": if motion is detected again immediately, it will go directly
    // to ALARM (without a new silent-arm cycle), since rtcDeviceState remains ARMED.
    rtcDeviceState = STATE_ARMED;
    publishStateTopic();
    sleepNow();
  }

  // mqttClient.loop();
}

void sleepNow() {
  detachInterrupt(digitalPinToInterrupt(WAKE_PIN));

  publishDeviceStatus(true); // sleeping = true
  publishBatteryStatus();

  modem.gprsDisconnect();
  GPSTurnOff();

  Serial.println("Shutting down modem to save power...");

  bool modemOff = false;
  for (int offTry = 0; offTry < 3 && !modemOff; offTry++) {
    if (modem.poweroff()) {
      Serial.println("Modem powered off!");
    } else {
      Serial.println("Modem power off failed, retrying...");
    }

    Serial.println("Check modem response.");
    int offAttempts = 0;
    while (modem.testAT() && offAttempts < 20) {
      Serial.print(".");
      delay(500);
      offAttempts++;
    }

    if (!modem.testAT()) {
      modemOff = true;
      Serial.println("Modem is not responding, modem has slept!");
    } else {
      Serial.println("Modem still responding after wait, retrying poweroff...");
    }
  }

  if (!modemOff) {
    Serial.println("WARNING: modem did not confirm power-off after retries, continuing to deep sleep anyway.");
  }

  delay(1000);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

#ifdef USE_MPU6050
  readMPU(0x3A); // INT_STATUS clear-on-read
  delay(50);

  int clearAttempts = 0;
  while (digitalRead(WAKE_PIN) == HIGH && clearAttempts < 5) {
    Serial.println("MPU INT still HIGH, clearing...");
    readMPU(0x3A);
    delay(50);
    clearAttempts++;
  }

  if (digitalRead(WAKE_PIN) == HIGH) {
    Serial.println("Warning: MPU INT is HIGH - the ESP32 might wake up immediately.");
  }
#endif
  // SW-420 NOTE: No clear latch or retry is needed here—the pin
  // returns to idle (LOW) on its own; there is no latch on the mechanical switch.

  gpio_num_t motionPin = static_cast<gpio_num_t>(WAKE_PIN);
  esp_sleep_enable_ext0_wakeup(motionPin, 1); // wake on HIGH

  SerialAT.end();
  btStop(); // Stop Bluetooth to save power
  delay(200);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

// ==================================================================
// fullPowerOff(): FULL, PERMANENT shutdown -- deep sleep WITHOUT any
// active wake source (neither ext0/motion nor timer). The device will NEVER
// wake up on its own again -- a physical reset or power cycle is required
// (e.g., disconnecting the battery, pressing the RESET button, or using the EN pin) for it to function again.
// Useful for complete shutdown (e.g., you sold the vehicle, service, etc.).
// ==================================================================
void fullPowerOff() {
  detachInterrupt(digitalPinToInterrupt(WAKE_PIN));
 
  publishDeviceStatus(true);
  modem.gprsDisconnect();
  GPSTurnOff();
 
  Serial.println("FULL shutdown of modem...");
  modem.poweroff();
  delay(1000);
 
  // NO wake source -- neither ext0, nor timer. Permanent sleep.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
 
  SerialAT.end();
  btStop();
  Serial.println("FULL shutdown of modem complete. A physical reset or power cycle is required for reactivation.");
  delay(200);
  esp_deep_sleep_start();
}

// ==================================================================
// sleepSilent(): short sleep for STATE_ARMED (no alert) --
// It does NOT touch the modem, because it was never activated during this cycle.
// ==================================================================
void sleepSilent() {
  detachInterrupt(digitalPinToInterrupt(WAKE_PIN));
 
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
 
#ifdef USE_MPU6050
  readMPU(0x3A);
  delay(50);
 
  int clearAttempts = 0;
  while (digitalRead(WAKE_PIN) == HIGH && clearAttempts < 5) {
    readMPU(0x3A);
    delay(50);
    clearAttempts++;
  }
#endif
 
  gpio_num_t motionPin = static_cast<gpio_num_t>(WAKE_PIN);
  esp_sleep_enable_ext0_wakeup(motionPin, 1);
 
  SerialAT.end(); // safe even if communication never started
  btStop();
  delay(100);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}