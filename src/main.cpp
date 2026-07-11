#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "modemManager.h"
#include "mqttManager.h"
#include <BLEDevice.h>
// #include <WiFi.h>
// #include <WiFiClient.h>

// ==================================================================
// SENSOR SELECTION — ενεργοποίησε ό,τι θες. Στο μέλλον μπορείς να έχεις
// και τους δύο ενεργούς ταυτόχρονα (π.χ. OR λογική στο interrupt) αν
// αποφασίσεις να τους συνδυάσεις — προς το παρόν, αν ενεργοποιήσεις και
// τους δύο, το WAKE_PIN παρακάτω θα δείχνει προσωρινά στον SW420_PIN.
// ==================================================================
#define USE_SW420
// #define USE_MPU6050

#ifdef USE_MPU6050
#include <Wire.h> // MPU6050-specific: χρειάζεται I2C
#endif

// === FUNCTIONS ===
void sleepNow();
void scanForKeyFob();
void IRAM_ATTR onMotionISR();

#ifdef USE_MPU6050
void writeMPU(uint8_t reg, uint8_t data); // MPU6050-specific (I2C)
uint8_t readMPU(uint8_t reg);              // MPU6050-specific (I2C)
void setupMPUMotionInterrupt();            // MPU6050-specific
#endif
#ifdef USE_SW420
void setupSW420MotionInterrupt();          // καθαρά digital, no I2C
#endif

// Ποιο pin χρησιμοποιείται τελικά για attachInterrupt / ext0 wakeup,
// ανάλογα ποιος αισθητήρας είναι ενεργός.
#if defined(USE_SW420) && defined(USE_MPU6050)
  #define WAKE_PIN SW420_PIN // και οι δύο ενεργοί - προσωρινά προτεραιότητα στον SW420, βλ. σχόλιο combine-both στο μέλλον
#elif defined(USE_SW420)
  #define WAKE_PIN SW420_PIN
#elif defined(USE_MPU6050)
  #define WAKE_PIN MOTION_INT_PIN
#else
  #error "You must enable at least one of the USE_SW420 / USE_MPU6050"
#endif

// ==== BLE Key Fob ====
BLEScan* pBLEScan;
bool keyFobFound = false;

// ==== Tracking ====
unsigned long sendInterval = 15000; // κάθε 15s (GPS/MQTT tracking) - μόνο σε FULL mode
unsigned long lastSend = 0;

unsigned long bleRescanInterval = 30000; // κάθε 30s επανέλεγχος BLE key fob όσο είμαστε ξύπνιοι (FULL mode)
unsigned long lastBleScan = 0;

unsigned long lastMotion = 0;

const unsigned long motionTimeout = 1UL * 60UL * 1000UL; // 1 λεπτό σε ms

// ==== Motion detection ====
// Το ISR δεν κάνει I2C calls (ασφαλές μέσα σε interrupt context).
// Απλά σηκώνει flag - Ο καθαρισμός γίνεται στο loop() (μόνο για MPU6050, βλ. εκεί).
volatile unsigned long lastMotionISR = 0;
volatile bool motionFlag = false;

void IRAM_ATTR onMotionISR() {
  unsigned long now = millis();
  if (now - lastMotionISR > 1000) { // hardware debounce 1000ms, μέσα στο ISR
    motionFlag = true;
    lastMotionISR = now;
  }
}

// ==================================================================
// Software "consensus" φίλτρο κίνησης — μειώνει false timer resets από
// μεμονωμένους κραδασμούς, χωρίς να αγγίζουμε το (ήδη πολύ ευαίσθητο)
// ποτενσιόμετρο του SW-420. Μόνο αν συμβούν αρκετά motion events μέσα
// σε ένα μικρό χρονικό παράθυρο θεωρούμε πραγματική, συνεχιζόμενη κίνηση
// (π.χ. οδήγηση) και κάνουμε reset το inactivity timer.
// ==================================================================
const unsigned long MOTION_WINDOW_MS = 5000;  // παράθυρο ανάλυσης (5s)
const int MOTION_CONSENSUS_COUNT = 3;         // ελάχιστα events μέσα στο παράθυρο
unsigned long motionWindowStart = 0;
int motionEventsInWindow = 0;

#ifdef USE_MPU6050
//////////////////////////////////////
// ---- MPU6050-specific I2C helpers: ΔΕΝ χρειάζονται καθόλου με τον SW-420 ----
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
  writeMPU(0x1C, 0x00);   // ACCEL_CONFIG: ±2g (μέγιστη ευαισθησία)
  // writeMPU(0x1C, 0x08);  // AFS_SEL=1 -> ±4g, λιγότερο ευαίσθητο σε μικροθόρυβο

  writeMPU(0x1F, 35);     // MOT_THR: motion threshold (~70mg). Ρύθμισέ το εμπειρικά.
  writeMPU(0x20, 30);     // MOT_DUR: motion duration ~30ms, φιλτράρει μεμονωμένα spikes

  writeMPU(0x69, 0x15);   // MOT_DETECT_CTRL

  writeMPU(0x37, 0x30);   // INT_PIN_CFG: active-high, push-pull, latch, clear-on-read

  writeMPU(0x6C, 0x87);   // STBY_XG/YG/ZG=1 (0x07) | LP_WAKE_CTRL=10 (20Hz) => 0x87
  writeMPU(0x6B, 0x20);   // PWR_MGMT_1: CYCLE=1
  delay(100);             // settle time πριν ενεργοποιήσουμε το interrupt

  writeMPU(0x38, 0x40);   // INT_ENABLE: motion interrupt enabled (οπλίζεται τελευταίο)

  readMPU(0x3A); // INT_STATUS (clear-on-read) - καθαρίζει τυχόν transient interrupt

  pinMode(MOTION_INT_PIN, INPUT);
}
////////////////////////////////////////
#endif // USE_MPU6050

#ifdef USE_SW420
// ==================================================================
// SW-420 setup.
// ΕΠΙΒΕΒΑΙΩΜΕΝΗ πολικότητα στο δικό σου module (μέσω sw420_test.cpp):
//   idle  = LOW
//   pulse = HIGH (πολύ σύντομο, μερικά ms, σε κάθε δόνηση/χτύπημα)
// Άρα: RISING edge για το interrupt, και ext0 wake on HIGH (1).
// Η ευαισθησία ρυθμίζεται ΜΟΝΟ με το ποτενσιόμετρο πάνω στο module.
// ==================================================================
void setupSW420MotionInterrupt() {
  pinMode(SW420_PIN, INPUT); // #define SW420_PIN <GPIO> στο config.h
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
  pBLEScan->clearResults(); // απελευθέρωση μνήμης μετά από κάθε scan

  sendKeyFobStatus(keyFobFound);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPTracer starting...");

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

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Normal boot");
  } else {
    Serial.println("Wakeup from EXT0 (motion)");
  }

  // Ενεργοποίηση interrupt ώστε να ανιχνεύσει κίνηση και ενώ είναι awake
  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), onMotionISR, RISING);

  // ==================================================================
  // === BLE key fob scan — γίνεται ΠΡΩΤΑ, πριν ανάψει το modem/GPS ===
  // Το BLE scan είναι στο ESP32 (χαμηλή κατανάλωση), ενώ το modem SIM7000
  // κοστίζει πολύ περισσότερο. Ελέγχοντας το keyfob πρώτα αποφασίζουμε ΑΝ
  // χρειάζεται καν πλήρες tracking session, πριν ξοδέψουμε ενέργεια.
  // ==================================================================
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(false); // Passive scan to save power
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  scanForKeyFob();
  lastBleScan = millis();

  // Αν βρέθηκε το keyfob, θέλουμε "light" mode: μία αναφορά θέσης πριν τον ύπνο, όχι συνεχές tracking.
  bool lightMode = keyFobFound;
  if (lightMode) {
    Serial.println("Keyfob found -> LIGHT mode (one location report before sleep).");
  } else {
    Serial.println("Keyfob not found -> FULL tracking mode.");
  }

  // Pull down DTR to ensure the modem is not in sleep state
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  // Power ON sequence for SIM7000
  modemPowerOn();
  delay(5000); // Wait for modem to start

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
  delay(500); // Wait for GPS to stabilize

  // Connect MQTT
  connectToMQTT();
  delay(500);

  sendDeviceStatus(false); // sleeping = false

  sendBatteryStatus();
  delay(500);

  sendModemStatus();
  delay(500);

  lastMotion = millis();
  motionWindowStart = millis();
  motionEventsInWindow = 0;

  // ==================================================================
  // LIGHT MODE: keyfob βρέθηκε -> μία αναφορά θέσης, μετά κατευθείαν sleep.
  // Δεν μπαίνουμε καθόλου στο loop() σε αυτή την περίπτωση.
  //
  // TRADE-OFF: αν το keyfob παραμένει «βρεθέν» καθ' όλη τη διάρκεια της
  // διαδρομής, το SW-420 θα ξυπνάει τη συσκευή σε κάθε κραδασμό/κίνηση,
  // και ΚΑΘΕ wake θα κάνει πλήρη κύκλο modem-on / GPS-fix / MQTT-send πριν
  // ξανακοιμηθεί. Αυτό μπορεί να καταναλώνει ΠΕΡΙΣΣΟΤΕΡΗ μπαταρία από το
  // συνεχές FULL tracking, γιατί το "άναμμα" του modem/GPRS κοστίζει πολύ.
  // Αν το δεις να αδειάζει γρήγορα η μπαταρία σε πραγματικό ταξίδι, πες μου
  // να προσθέσουμε ένα cooldown (π.χ. min 5-10 λεπτά ανάμεσα σε reports).
  // ==================================================================
  if (lightMode) {
    float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
    int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

    Serial.println("LIGHT mode: requesting one-shot location before sleep...");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
      &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      sendLocation(lat, lon, alt, speed, accuracy);
    } else {
      Serial.println("LIGHT mode: no GPS fix found, sleeping without location.");
    }
    delay(500);

    sleepNow(); // deep sleep — δεν επιστρέφει, το loop() δεν τρέχει καθόλου σε αυτόν τον κύκλο
    return;     // φρουρός, ποτέ δεν φτάνει εδώ στην πράξη
  }

  // FULL mode: συνεχίζουμε κανονικά, το loop() θα αναλάβει το tracking
}

void loop() {
  unsigned long now = millis();

  // Update last motion time if motion detected — με software consensus φίλτρο
  if (motionFlag) {
    motionFlag = false;
#ifdef USE_MPU6050
    readMPU(0x3A); // INT_STATUS: clear-on-read -- MPU6050-specific
#endif
    // SW-420 NOTE: δεν χρειάζεται κανένα clear-on-read - το DO pin δεν κάνει latch.
    if (now - motionWindowStart > MOTION_WINDOW_MS) {
      motionWindowStart = now;
      motionEventsInWindow = 1;
    } else {
      motionEventsInWindow++;
    }

    if (motionEventsInWindow >= MOTION_CONSENSUS_COUNT) {
      lastMotion = now; // "πραγματική" συνεχιζόμενη κίνηση επιβεβαιωμένη
      Serial.println("Confirmed motion (consensus) - Timer reset.");
    } else {
      Serial.println("Motion event (" + String(motionEventsInWindow) + "/" +
                      String(MOTION_CONSENSUS_COUNT) + ") - waiting for consensus.");
    }
  }

  // === BLE rescan - περιοδικός επανέλεγχος όσο παραμένουμε ξύπνιοι ===
  if (now - lastBleScan >= bleRescanInterval) {
    lastBleScan = now;
    Serial.println("Re-scanning for key fob...");
    scanForKeyFob();
    // Σημείωση: εδώ θα μπορούσαμε στο μέλλον να μεταβούμε από FULL σε LIGHT
    // αν το keyfob ξαναβρεθεί μέσα σε ένα FULL tracking session.
  }

  // === GPS === (μόνο σε FULL mode, αφού το loop() τρέχει μόνο τότε)
  if (now - lastSend >= sendInterval) {
    lastSend = now;

    float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
    int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

    Serial.println("Requesting current location");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
      &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      sendLocation(lat, lon, alt, speed, accuracy);
    } else {
      Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in " + String(sendInterval / 1000) + "s.");
    }
  }

  // === Check inactivity ===
  if (now - lastMotion > motionTimeout) {
    Serial.println("Stop - No motion for " + String(motionTimeout / 1000) + " seconds.");
    sleepNow();
  }

  mqttClient.loop();
}

void sleepNow() {
  detachInterrupt(digitalPinToInterrupt(WAKE_PIN));

  sendDeviceStatus(true); // sleeping = true
  sendBatteryStatus();

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
  // SW-420 NOTE: δεν χρειάζεται κανένα clear latch / retry εδώ - το pin
  // επιστρέφει μόνο του στο idle (LOW), δεν υπάρχει latch σε mechanical switch.

  gpio_num_t motionPin = static_cast<gpio_num_t>(WAKE_PIN);
  esp_sleep_enable_ext0_wakeup(motionPin, 1); // wake on HIGH

  SerialAT.end();
  btStop(); // Stop Bluetooth to save power
  delay(200);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}