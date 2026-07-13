// ============================================================
// SW-420 TEST SKETCH
// Σκοπός: να δεις στο Serial Monitor
//   1) ποιο είναι το idle state του DO pin (HIGH ή LOW όταν ΔΕΝ κινείται τίποτα)
//   2) τι γίνεται τη στιγμή που χτυπάς/κουνάς τον αισθητήρα (pulse HIGH ή LOW)
//   3) πόσο "ευαίσθητο" είναι με το τρέχον setting του ποτενσιόμετρου
//
// Σύνδεση SW-420 -> ESP32:
//   VCC -> 3V3 (ή 5V αν το module σου το θέλει - έλεγξε το δικό σου board)
//   GND -> GND
//   DO  -> οποιοδήποτε GPIO (πχ 32)
// ============================================================

#include <Arduino.h>

#define SW420_PIN 32   // <-- άλλαξέ το στο GPIO που θα χρησιμοποιήσεις τελικά

volatile bool motionFlag = false;
volatile unsigned long lastISR = 0;

void IRAM_ATTR onMotionISR() {
  unsigned long now = millis();
  if (now - lastISR > 50) { // απλό debounce μόνο για το test
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

  // Δοκιμάζουμε ΧΩΡΙΣ pull-up πρώτα, μόνο διάβασμα raw τιμής,
  // ώστε να δούμε το πραγματικό idle state του module.
  pinMode(SW420_PIN, INPUT);

  int initial = digitalRead(SW420_PIN);
  Serial.print("Αρχική (idle) κατάσταση pin: ");
  Serial.println(initial == HIGH ? "HIGH" : "LOW");
  Serial.println("Αν το module σου έχει ενσωματωμένο pull-up/pull-down, αυτή η τιμή");
  Serial.println("δείχνει το πραγματικό idle state. Αν βλέπεις τυχαία/ασταθή τιμή,");
  Serial.println("δοκίμασε INPUT_PULLUP παρακάτω στον κώδικα.");
  Serial.println();

  // Attach interrupt και στις δύο ακμές, ώστε να δούμε ό,τι συμβαίνει
  attachInterrupt(digitalPinToInterrupt(SW420_PIN), onMotionISR, CHANGE);

  Serial.println("Ξεκίνα να χτυπάς/κουνάς ελαφρά τον αισθητήρα...");
  Serial.println("Η στήλη 'state' δείχνει live την τιμή του pin κάθε 200ms.");
  Serial.println("Η γραμμή '>>> INTERRUPT <<<' εμφανίζεται όποτε πιάνεται edge trigger.");
  Serial.println();
}

void loop() {
  unsigned long now = millis();

  // Live polling της κατάστασης κάθε 200ms, για να δεις το idle level
  // και πόσο "τρεμοπαίζει" ο αισθητήρας ακόμα και χωρίς να τον αγγίζεις
  // (αυτό δείχνει πόσο σωστά είναι ρυθμισμένο το ποτενσιόμετρο ευαισθησίας).
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
    Serial.print("ms, τρέχουσα τιμή pin = ");
    Serial.print(digitalRead(SW420_PIN) == HIGH ? "HIGH" : "LOW");
    Serial.println(")");
  }
}

// ============================================================
// ΠΩΣ ΝΑ ΔΙΑΒΑΣΕΙΣ ΤΑ ΑΠΟΤΕΛΕΣΜΑΤΑ:
//
// 1) Άφησέ το ήσυχο για μερικά δευτερόλεπτα μετά το boot.
//    - Αν το idle state είναι σταθερό HIGH -> το module σου είναι idle=HIGH,
//      δηλαδή θα χρησιμοποιήσεις FALLING edge + ext0 wake on LOW (0) στο τελικό project.
//    - Αν το idle state είναι σταθερό LOW -> idle=LOW,
//      θα χρησιμοποιήσεις RISING edge + ext0 wake on HIGH (1).
//    - Αν "τρεμοπαίζει" μόνο του χωρίς να το αγγίζεις -> το ποτενσιόμετρο είναι
//      πολύ ευαίσθητο, γύρισέ το (συνήθως δεξιόστροφα = πιο ευαίσθητο, αριστερόστροφα
//      = λιγότερο, αλλά διαφέρει ανά module) μέχρι να σταθεροποιηθεί.
//
// 2) Χτύπα/κούνα ελαφρά τον αισθητήρα:
//    - Δες αν εμφανίζεται ">>> INTERRUPT <<<" εύκολα με ήπιο χτύπημα
//      ή μόνο με δυνατό χτύπημα -> ρύθμισε το ποτενσιόμετρο ανάλογα.
//
// 3) Μόλις καταλάβεις πολικότητα + σωστό σημείο ποτενσιόμετρου, πήγαινε στο
//    setupSW420MotionInterrupt() στο main.cpp και βάλε τη σωστή λογική
//    (FALLING/RISING, ext0 wake 0/1) όπως σημειώθηκε εκεί.
// ============================================================