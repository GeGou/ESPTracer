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
void sleepSilent();
void scanForKeyFob();
void IRAM_ATTR onMotionISR();
void disarmedFlow();
void startAlarmTracking();
// void publishAlarmEvent();
// void publishStateTopic();
void fullPowerOff();

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


// ==================================================================
// STATE MACHINE (Monimoto-style): DISARMED / ARMED / ALARM
//   DISARMED : keyfob βρέθηκε -> μία αναφορά θέσης, sleep. Καμία ειδοποίηση.
//   ARMED    : keyfob ΔΕΝ βρέθηκε, ΠΡΩΤΗ φορά -> silent arming, ΚΑΝΕΝΑ modem,
//              πάει κατευθείαν για ύπνο, οπλισμένο.
//   ALARM    : ήδη ήταν ARMED και ξανάρθε motion event -> escalation, πλήρες
//              tracking + άμεση ειδοποίηση.
// Το rtcDeviceState επιβιώνει το deep sleep μέσω RTC memory (RTC_DATA_ATTR).
// ==================================================================
// enum DeviceState { 
//   STATE_DISARMED = 0,
//   STATE_ARMED = 1,
//   STATE_ALARM = 2 
// };

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

// Software consensus φίλτρο κίνησης (μόνο για ΟΣΟ διαρκεί το ALARM tracking loop)
unsigned long motionWindowStart = 0;
int motionEventsInWindow = 0;

// MQTT commands (μέσω MQTT_TOPIC_COMMAND) -- λειτουργούν ΜΟΝΟ όσο η
// συσκευή είναι ήδη ξύπνια/συνδεδεμένη (δηλαδή κατά τη διάρκεια ALARM).
volatile bool stopAlarmRequested = false;
volatile bool powerOffRequested = false;

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
// ΕΠΙΒΕΒΑΙΩΣΗ πολικότητας στο module μέσω sw420_test.cpp:
// π.χ  idle  = LOW
//      pulse = HIGH (πολύ σύντομο, μερικά ms, σε κάθε δόνηση/χτύπημα)
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
  publishKeyFobStatus(keyFobFound);
}
 
// ==================================================================
// Κοινό "άναμμα" modem/GPRS/GPS/MQTT -- χρησιμοποιείται και από DISARMED
// (μία αναφορά) και από ALARM (πλήρες tracking).
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
 
  // Check GPRS connectio
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

  // NEW: εγγραφή για λήψη εντολών (STOP_ALARM / POWER_OFF) -- δουλεύει μόνο
  // όσο η συσκευή παραμένει ξύπνια/συνδεδεμένη (δηλαδή κατά τη διάρκεια ALARM).
  mqttClient.setCallback(callback);
  mqttClient.subscribe(MQTT_TOPIC_COMMAND);
}
 
// ==================================================================
// DISARMED flow: μία αναφορά θέσης πριν sleep. Καμία επαναλαμβανόμενη
// αποστολή, δεν μπαίνει στο loop().
// ==================================================================
void disarmedFlow() {
  powerUpConnectivity();
 
  publishDeviceStatus(false);
  publishBatteryStatus();
  delay(500);
  publishModemStatus();
  delay(500);
 
  publishStateTopic(); // ενημερώνει αν μόλις επέστρεψε από ARMED/ALARM
 
  float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
  int   vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
 
  Serial.println("DISARMED: requesting one-shot location before sleep...");
  if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat,
    &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
    publishLocation(lat, lon, alt, speed, accuracy);
  } else {
    Serial.println("DISARMED: δεν βρέθηκε GPS fix, sleep χωρίς θέση.");
  }
  delay(500);
 
  sleepNow();
}
 
// ==================================================================
// ALARM tracking: πλήρες άναμμα + ΑΜΕΣΗ ειδοποίηση πριν καν περιμένουμε
// GPS fix, μετά συνεχίζει σαν το παλιό FULL mode μέσα στο loop().
// ==================================================================
void startAlarmTracking() {
  powerUpConnectivity();
 
  publishStateTopic();
  publishAlarmEvent(); // ΠΡΩΤΑ η ειδοποίηση, πριν περιμένουμε GPS fix
 
  publishDeviceStatus(false);
  publishBatteryStatus();
  delay(500);
  publishModemStatus();
  delay(500);
 
  lastSend = 0; // ώστε το loop() να στείλει GPS fix αμέσως στον πρώτο κύκλο
}
/////////////////////////////////////////////////////


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

  bool normalBoot = false;
  if (cause != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Normal boot");
    normalBoot = true;
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
  bool keyfobPresent = keyFobFound;
  bool effectiveArmed = !keyfobPresent;

  DeviceState newState;
  if (!effectiveArmed) {
    newState = STATE_DISARMED;
  } else if (normalBoot) {
    // Πρώτη εκκίνηση/reset ενώ armed -> ξεκινάμε καθαρά από ARMED, ποτέ ALARM.
    newState = STATE_ARMED;
  } else if (rtcDeviceState == STATE_ARMED || rtcDeviceState == STATE_ALARM) {
    // Ήδη armed από πριν, ΝΕΟ motion event -> escalation σε ALARM
    // (2ο+ διαδοχικό motion event ενώ armed = πραγματική κίνηση, όχι απλά
    // ο ιδιοκτήτης που απομακρύνεται μία φορά).
    newState = STATE_ALARM;
  } else {
    // Πρώτη φορά armed (μόλις έφυγε το keyfob) -> silent arming.
    newState = STATE_ARMED;
  }

  Serial.println("State: " + String(stateNameOf(rtcDeviceState)) + " -> " + String(stateNameOf(newState)));
  rtcDeviceState = newState;

  lastMotion = millis();
  motionWindowStart = millis();
  motionEventsInWindow = 0;

  if (newState == STATE_DISARMED) {
    disarmedFlow();
    return; // δεν φτάνει ποτέ εδώ -- disarmedFlow() κάνει sleepNow()
  }
 
  if (newState == STATE_ARMED) {
    Serial.println("ARMED (silent) -- κανένα modem, ξανά για ύπνο.");
    sleepSilent();
    return; // δεν φτάνει ποτέ εδώ
  }
 
  // STATE_ALARM
  startAlarmTracking(); // επιστρέφει κανονικά, το loop() αναλαμβάνει το tracking

}

void loop() {
  unsigned long now = millis();
  
  mqttClient.loop(); // ΠΡΕΠΕΙ να τρέχει ώστε να φτάνουν οι εντολές στο callback()

  // Έλεγχος για εντολές που ήρθαν μέσω MQTT_TOPIC_COMMAND
  if (powerOffRequested) {
    Serial.println("Εντολή POWER_OFF ελήφθη -- πλήρης, μόνιμη απενεργοποίηση.");
    fullPowerOff();
    return; // δεν φτάνει ποτέ εδώ
  }
 
  if (stopAlarmRequested) {
    Serial.println("Εντολή STOP_ALARM ελήφθη -- σταματάει το τρέχον alarm.");
    stopAlarmRequested = false;
    rtcDeviceState = STATE_ARMED; // παραμένει "άγρυπνο", θα ξανασκάσει σε νέα κίνηση
    publishStateTopic();
    sleepNow();
    return; // δεν φτάνει ποτέ εδώ
  }

  // Αυτό το loop() τρέχει ΜΟΝΟ όσο rtcDeviceState == STATE_ALARM.
  // Update last motion time if motion detected — με software consensus φίλτρο
  if (motionFlag) {
    motionFlag = false;
#ifdef USE_MPU6050
    readMPU(0x3A); // INT_STATUS: clear-on-read -- MPU6050-specific
#endif
    // FIX: sliding λογική -- μηδενισμός ΜΟΝΟ αν υπάρξει πραγματικό κενό (>MOTION_WINDOW_MS)
    // από το ΠΡΟΗΓΟΥΜΕΝΟ event, όχι από την αρχή ενός σταθερού παραθύρου. Έτσι, συνεχόμενα
    // events (π.χ. 1/sec λόγω hardware debounce) δεν "χάνουν" ποτέ το count λόγω απόλυτου
    // χρόνου -- μόνο μια πραγματική παύση στην κίνηση κάνει reset.    
    if (now - motionWindowStart > MOTION_WINDOW_MS) {
      motionEventsInWindow = 1;
    } else {
      motionEventsInWindow++;
    }
    motionWindowStart = now;  // ενημέρωση σε ΚΑΘΕ event, όχι μόνο στο reset

    if (motionEventsInWindow >= MOTION_CONSENSUS_COUNT) {
      lastMotion = now; // "πραγματική" συνεχιζόμενη κίνηση επιβεβαιωμένη
      Serial.println("Confirmed motion (consensus) - Timer reset.");
    } else {
      Serial.println("Motion event (" + String(motionEventsInWindow) + "/" +
                      String(MOTION_CONSENSUS_COUNT) + ") - waiting for consensus.");
    }
  }

  // === BLE rescan - περιοδικός επανέλεγχος όσο παραμένουμε ξύπνιοι ===
  if (now - lastBleScan >= BLE_RESCAN_INTERVAL_MS) {
    lastBleScan = now;
    Serial.println("Re-scanning for key fob...");
    scanForKeyFob();
    // Σημείωση: εδώ θα μπορούσαμε στο μέλλον να μεταβούμε από FULL σε LIGHT
    // αν το keyfob ξαναβρεθεί μέσα σε ένα FULL tracking session.
    if (keyFobFound) {
      Serial.println("🔑 Keyfob επέστρεψε κατά τη διάρκεια ALARM -> DISARMED, τερματισμός.");
      rtcDeviceState = STATE_DISARMED;
      publishStateTopic();
      sleepNow();
      return;
    }
  }

  // === GPS === (μόνο σε FULL mode, αφού το loop() τρέχει μόνο τότε)
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
    // Παραμένει "προετοιμασμένο": αν ξαναρθεί motion αμέσως, θα πάει κατευθείαν
    // σε ALARM (χωρίς νέο silent-arm κύκλο), αφού rtcDeviceState μένει ARMED.
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

// ==================================================================
// fullPowerOff(): ΠΛΗΡΗΣ, ΜΟΝΙΜΗ απενεργοποίηση -- deep sleep ΧΩΡΙΣ κανένα
// wake source ενεργό (ούτε ext0/motion, ούτε timer). Η συσκευή ΔΕΝ θα
// ξυπνήσει ποτέ μόνη της ξανά -- χρειάζεται φυσικό reset ή power-cycle
// (π.χ. αποσύνδεση μπαταρίας, κουμπί RESET, ή EN pin) για να ξαναλειτουργήσει.
// Χρήσιμο για πλήρη απενεργοποίηση (π.χ. πούλησες το όχημα, service κλπ.).
// ==================================================================
void fullPowerOff() {
  detachInterrupt(digitalPinToInterrupt(WAKE_PIN));
 
  publishDeviceStatus(true);
  modem.gprsDisconnect();
  GPSTurnOff();
 
  Serial.println("Πλήρης απενεργοποίηση modem...");
  modem.poweroff();
  delay(1000);
 
  // ΚΑΝΕΝΑ wake source -- ούτε ext0, ούτε timer. Μόνιμος ύπνος.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
 
  SerialAT.end();
  btStop();
  Serial.println("Η συσκευή απενεργοποιείται ΜΟΝΙΜΑ. Χρειάζεται φυσικό reset/power-cycle για επανεκκίνηση.");
  delay(200);
  esp_deep_sleep_start();
}

// ==================================================================
// sleepSilent(): ελαφρύ sleep για STATE_ARMED (χωρίς ειδοποίηση) --
// ΔΕΝ αγγίζει το modem, γιατί ποτέ δεν ενεργοποιήθηκε σε αυτόν τον κύκλο.
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
 
  SerialAT.end(); // ασφαλές ακόμα κι αν δεν ξεκίνησε ποτέ επικοινωνία
  btStop();
  delay(100);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}