#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "modemManager.h"
#include "mqttManager.h"
#include <BLEDevice.h>
#include <Wire.h>
// #include <WiFi.h>
// #include <WiFiClient.h>


// // WiFiClient wifiClient;

// // PubSubClient mqttClient(wifiClient);

// === FUNCTIONS ===
void sleepNow();
void scanForKeyFob();
void writeMPU(uint8_t reg, uint8_t data);
uint8_t readMPU(uint8_t reg);
void setupMPUMotionInterrupt();
void IRAM_ATTR onMotionISR();

// ==== BLE Key Fob ====
BLEScan* pBLEScan;
bool keyFobFound = false;

// ==== Tracking ====
unsigned long sendInterval = 15000; // κάθε 15s (GPS/MQTT tracking)
unsigned long lastSend = 0;

unsigned long bleRescanInterval = 30000; // κάθε 30s επανέλεγχος BLE key fob όσο είμαστε ξύπνιοι
unsigned long lastBleScan = 0;

unsigned long lastMotion = 0;

// FIX #1: to motionTimeout συγκρίνεται με millis() (ms), όχι micros().
// Το προηγούμενο "2 * 60 * uS_TO_S_FACTOR" έκανε το timeout ~33 ώρες αντί για 2 λεπτά.
const unsigned long motionTimeout = 1UL * 60UL * 1000UL; // 1 λεπτό σε ms

// ==== Motion detection (MPU6050) ====
// Το ISR δεν κάνει I2C calls (ασφαλές μέσα σε interrupt context).
// Απλά σηκώνει flag· ο καθαρισμός του MPU latch (I2C read) γίνεται στο loop().
volatile unsigned long lastMotionISR = 0;
volatile bool motionFlag = false;

void IRAM_ATTR onMotionISR() {
  unsigned long now = millis();
  if (now - lastMotionISR > 1000) { // debounce 1000ms
    motionFlag = true;
    lastMotionISR = now;
  }
}

//////////////////////////////////////
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
  // FIX #8 (immediate re-wake bug): η σειρά έχει αλλάξει ώστε το power-mode transition
  // (cycle mode) να ολοκληρωθεί και να "settle" ΠΡΙΝ ενεργοποιήσουμε το motion interrupt.
  // Το να ανάβεις το interrupt ενώ το accel μόλις άλλαξε power mode είναι ό,τι προκαλούσε
  // ένα ψευδές motion event να κολλήσει latched στο INT pin, με αποτέλεσμα το ext0
  // (level-triggered) να ξυπνάει το ESP32 αμέσως μόλις έμπαινε σε deep sleep.

  writeMPU(0x6B, 0x00);   // PWR_MGMT_1: exit sleep, internal 8MHz osc
  delay(10);

  writeMPU(0x6C, 0x07);   // PWR_MGMT_2: STBY_XG/YG/ZG=1 -> gyro OFF, accel only (FIX #7, μέρος 1)
  writeMPU(0x1C, 0x00);   // ACCEL_CONFIG: ±2g (μέγιστη ευαισθησία)

  writeMPU(0x1F, 10);     // MOT_THR: motion threshold (~320mg). Ρύθμισέ το εμπειρικά.
  writeMPU(0x20, 80);     // MOT_DUR: motion duration ~80ms, φιλτράρει μεμονωμένα spikes

  writeMPU(0x69, 0x15);   // MOT_DETECT_CTRL

  // FIX #5 (διορθωμένο): INT_PIN_CFG. Η τιμή 0xA0 (bit7=1) ήταν λάθος -> INT_LEVEL=1
  // σημαίνει active-LOW, δηλαδή το pin idle-άρει HIGH μόνιμα και πέφτει LOW μόνο όσο
  // διαρκεί το interrupt. Αυτό έκανε το ext0 (wake on HIGH) να ξυπνάει αμέσως, αφού
  // το idle state ήταν ήδη HIGH ανεξαρτήτως πραγματικής κίνησης.
  // Σωστή τιμή 0x30 = 0b00110000: INT_LEVEL=0 (active-high, idle LOW), INT_OPEN=0
  // (push-pull), LATCH_INT_EN=1 (μένει HIGH μέχρι clear), INT_RD_CLEAR=1 (clear σε
  // οποιοδήποτε read).
  writeMPU(0x37, 0x30);

  // FIX #7, μέρος 2: Cycle mode - accel-only low power sampling (CYCLE bit).
  // Ενεργοποιείται ΠΡΙΝ το INT_ENABLE, και αφήνουμε χρόνο να σταθεροποιηθεί η
  // δειγματοληψία πριν οπλίσουμε το interrupt (FIX #8).
  // LP_WAKE_CTRL (bits 7:6 του 0x6C): 00=1.25Hz, 01=5Hz, 10=20Hz, 11=40Hz.
  writeMPU(0x6C, 0x87);   // STBY_XG/YG/ZG=1 (0x07) | LP_WAKE_CTRL=10 (20Hz) => 0x87
  writeMPU(0x6B, 0x20);   // PWR_MGMT_1: CYCLE=1
  delay(100);             // FIX #8: settle time πριν ενεργοποιήσουμε το interrupt

  writeMPU(0x38, 0x40);   // INT_ENABLE: motion interrupt enabled (οπλίζεται τελευταίο)

  // Clear τυχόν transient interrupt που προκλήθηκε από τη μετάβαση config/power-mode
  readMPU(0x3A); // INT_STATUS (clear-on-read)

  pinMode(MOTION_INT_PIN, INPUT); // INT pin (push-pull, δεν χρειάζεται pull-up/down)
}
////////////////////////////////////////

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
  pBLEScan->clearResults(); // απελευθέρωση μνήμης μετά από κάθε scan

  sendKeyFobStatus(keyFobFound);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPTracer starting...");

  Wire.begin(21, 22); // SDA, SCL
  setupMPUMotionInterrupt();

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wakeup cause: %d\n", cause);

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Normal boot");
  } else {
    Serial.println("Wakeup from EXT0 (motion)");
  }

  // Ενεργοποίηση interrupt ώστε να ανιχνεύουμε κίνηση και ενώ είμαστε ξύπνιοι
  attachInterrupt(digitalPinToInterrupt(MOTION_INT_PIN), onMotionISR, RISING);

  // Pull down DTR to ensure the modem is not in sleep state
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  // Power ON sequence for SIM7000
  modemPowerOn();
  delay(5000); // Wait for modem to start

  // Ο βρόχος επαναλαμβάνεται, κάνει modem.restart() κάθε 10 αποτυχίες, και μόνο
  // μετά από 20 αποτυχίες κάνει ESP.restart().
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
      delay(3000); // Wait for modem to restart
    }

    if (attempts > 20) {
      Serial.println("Modem still not responding after restart, restarting ESP32!");
      ESP.restart();
    }

    modemOK = modem.testAT();
  }
  Serial.println("Modem is online!");

  // Set LED OFF
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

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
    checkModemStatus();   // Need to check that function
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
  delay(500); // Wait for GPS to stabilize

  // Connect MQTT
  connectToMQTT();
  delay(500);

  // μόλις συνδεθούμε, δηλώνουμε στο MQTT ότι η συσκευή είναι ξύπνια/tracking
  sendDeviceStatus(false); // sleeping = false

  // === BLE key fob scan - πρώτος έλεγχος κατά την αφύπνιση ===
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(false); // Passive scan to save power
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  scanForKeyFob();
  lastBleScan = millis();
  delay(500);

  // Battery status every time ESP wakes up
  sendBatteryStatus();
  delay(500);

  // Modem status every time ESP wakes up
  sendModemStatus();
  delay(500);

  lastMotion = millis();
}

void loop() {
  unsigned long now = millis();

  // Update last motion time if motion detected
  if (motionFlag) {
    motionFlag = false;
    readMPU(0x3A); // INT_STATUS: clear-on-read, ξεκλειδώνει το latch για το επόμενο event
    lastMotion = now;
    Serial.println("🟡 Motion detected! Timer reset.");
  }

  // === BLE rescan - περιοδικός επανέλεγχος όσο παραμένουμε ξύπνιοι ===
  if (now - lastBleScan >= bleRescanInterval) {
    lastBleScan = now;
    Serial.println("Re-scanning for key fob...");
    scanForKeyFob();
  }

  // === GPS ===
  if (now - lastSend >= sendInterval) {
    lastSend = now;

    // Read GPS location and send it over MQTT
    float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
    int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

    Serial.println("Requesting current location");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
      &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {

      // Send over MQTT
      sendLocation(lat, lon, alt, speed, accuracy);
    } else {
      Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in " + String(sendInterval / 1000) + "s.");
    }
  }

  // === Check inactivity ===
  if (now - lastMotion > motionTimeout) {
    Serial.println("Stop No motion for " + String(motionTimeout / 1000) + " seconds.");
    sleepNow();
  }

  mqttClient.loop();
}

void sleepNow() {
  detachInterrupt(digitalPinToInterrupt(MOTION_INT_PIN));

  // δηλώνουμε "sleeping" στο MQTT ΠΡΙΝ κλείσουμε modem/GPRS - αλλιώς δεν
  // προλαβαίνει να φύγει το μήνυμα, αφού μετά χάνεται η σύνδεση. Η ίδια η
  // sendDeviceStatus() κάνει ήδη mqttClient.loop()+delay(1000)+loop() εσωτερικά
  // για να δώσει χρόνο στο modem να ολοκληρώσει την αποστολή.
  sendDeviceStatus(true); // sleeping = true

  // Battery status before sleep
  sendBatteryStatus();

  // Shutdown modem and GPS to save power
  modem.gprsDisconnect();
  GPSTurnOff();

  Serial.println("Shutting down modem to save power...");

  // Κάνουμε έως 3 προσπάθειες poweroff, με bounded wait στην καθεμία, και αν όλες 
  // αποτύχουν προχωράμε στο deep sleep όπως και να 'χει.
  // (καλύτερα να προσπαθήσουμε ξανά στον επόμενο κύκλο, παρά να μείνουμε κολλημένοι).
  bool modemOff = false;
  for (int offTry = 0; offTry < 3 && !modemOff; offTry++) {
    if (modem.poweroff()) {
      Serial.println("Modem powered off!");
    } else {
      Serial.println("Modem power off failed, retrying...");
    }

    Serial.println("Check modem response.");
    int offAttempts = 0;
    while (modem.testAT() && offAttempts < 20) { // μέγιστη αναμονή ~10s ανά προσπάθεια
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

  // Prepare for wake on motion (MPU6050 INT pin)
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // (immediate re-wake bug): ΔΕΝ ξανατρέχουμε setupMPUMotionInterrupt() εδώ.
  // Το MPU6050 τροφοδοτείται ανεξάρτητα από το ESP32 και κρατάει ήδη τη ρύθμιση από
  // το setup()· το να ξαναγράφεις PWR_MGMT_1/2 (cycle mode) ακριβώς πριν τον ύπνο είναι
  // ό,τι προκαλούσε ένα ψευδές motion event να μείνει latched στο INT pin, με αποτέλεσμα
  // το ext0 (level-triggered) να ξυπνάει το ESP32 αμέσως.
  // Αρκεί να καθαρίσουμε το latch και να επιβεβαιώσουμε ότι το pin είναι πραγματικά LOW.
  readMPU(0x3A); // INT_STATUS clear-on-read
  delay(50);

  int clearAttempts = 0;
  while (digitalRead(MOTION_INT_PIN) == HIGH && clearAttempts < 5) {
    Serial.println("MPU INT ακόμα HIGH, ξανακαθαρίζω...");
    readMPU(0x3A);
    delay(50);
    clearAttempts++;
  }

  if (digitalRead(MOTION_INT_PIN) == HIGH) {
    Serial.println("ΠΡΟΣΟΧΗ: MPU INT παραμένει HIGH - το ESP32 πιθανόν να ξυπνήσει αμέσως.");
  }

  gpio_num_t motionPin = static_cast<gpio_num_t>(MOTION_INT_PIN);
  esp_sleep_enable_ext0_wakeup(motionPin, 1);

  SerialAT.end();
  btStop(); // Stop Bluetooth to save power
  delay(200);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}