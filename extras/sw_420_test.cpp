// ============================================================
// SW-420 TEST SKETCH
// Purpose: to see in the Serial Monitor
//   1) what the idle state of the DO pin is (HIGH or LOW when NOTHING is moving)
//   2) what happens when you tap or shake the sensor (HIGH or LOW pulse)
//   3) how “sensitive” it is with the current potentiometer setting
//
// SW-420 to ESP32 connection:
//   VCC -> 3V3 (or 5V if your module requires it—check your board)
//   GND -> GND
//   DO  -> any GPIO (e.g., 32)
// ============================================================

#include <Arduino.h>

#define SW420_PIN 32   // <-- change it to the GPIO you will use eventually

volatile bool motionFlag = false;
volatile unsigned long lastISR = 0;

void IRAM_ATTR onMotionISR() {
  unsigned long now = millis();
  if (now - lastISR > 50) { // simple debounce only for testing
    motionFlag = true;
    lastISR = now;
  }
}

unsigned long lastPrint = 0;
int lastState = -1;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== SW-420 TEST ===");

  // We test without pull-up first, just reading the raw value,
  // to see the actual idle state of the module.
  // pinMode(SW420_PIN, INPUT);
  pinMode(SW420_PIN, INPUT_PULLUP); // if your module has an integrated pull-up/pull-down, try this

  int initial = digitalRead(SW420_PIN);
  Serial.print("Initial (idle) pin state: ");
  Serial.println(initial == HIGH ? "HIGH" : "LOW");
  Serial.println("If your module has an integrated pull-up/pull-down, this value");
  Serial.println("indicates the actual idle state. If you see random/unstable values,");
  Serial.println("try INPUT_PULLUP below in the code.");
  Serial.println();

  // Attach interrupt και στις δύο ακμές, ώστε να δούμε ό,τι συμβαίνει
  attachInterrupt(digitalPinToInterrupt(SW420_PIN), onMotionISR, CHANGE);

  Serial.println("Start tapping/shaking the sensor lightly...");
  Serial.println("The 'state' column shows the live value of the pin every 200ms.");
  Serial.println("The line '>>> INTERRUPT <<<' appears when an edge trigger is detected.");
  Serial.println();
}

void loop() {
  unsigned long now = millis();

  // Live polling of the status every 200 ms, to see the idle level
  // and how much the sensor "flickers" even without touching it
  // (this indicates how well the sensitivity potentiometer is adjusted).
  if (now - lastPrint >= 200) {
    lastPrint = now;
    int state = digitalRead(SW420_PIN);
    if (state != lastState) {
      Serial.print("[t=");
      Serial.print(now);
      Serial.print("ms] state change -> ");
      Serial.println(state == HIGH ? "HIGH" : "LOW");
      lastState = state;
    }
  }

  if (motionFlag) {
    motionFlag = false;
    Serial.print(">>> INTERRUPT <<<  (t=");
    Serial.print(now);
    Serial.print("ms, current pin value = ");
    Serial.print(digitalRead(SW420_PIN) == HIGH ? "HIGH" : "LOW");
    Serial.println(")");
  }
}

// ============================================================
// How to read the results:
//
// 1) Leave it idle for a few seconds after boot.
//    - If the idle state is stable HIGH -> your module is idle=HIGH,
//      in other words, you will use FALLING edge + ext0 wake on LOW (0) in the final project.
//    - If the idle state is a steady LOW -> idle=LOW,
//      you’ll use a RISING edge + ext0 wake on HIGH (1).
//    - If it “flickers” on its own without you touching it -> the potentiometer is
//      very sensitive; turn it (usually clockwise = more sensitive, counterclockwise
// = less sensitive, but this varies by module) until it stabilizes.
//
// 2) Tap or gently shake the sensor:
//    - See if “>>> INTERRUPT <<<” appears easily with a gentle tap
//      or only with a strong tap -> adjust the potentiometer accordingly.
//
// 3) Once you’ve determined the polarity and the correct potentiometer setting, go to
//    setupSW420MotionInterrupt() in main.cpp and implement the correct logic
//    (FALLING/RISING, ext0 wake 0/1) as noted there.
// ============================================================