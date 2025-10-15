#include <Servo.h>
#include <Wire.h>

// ===== Per-servo pulse windows (µs) =====
static int SERVO1_US_MIN = 600,  SERVO1_US_MAX = 2400;
static int SERVO2_US_MIN = 600,  SERVO2_US_MAX = 2400;
static int SERVO3_US_MIN = 600,  SERVO3_US_MAX = 2400;
static int SERVO4_US_MIN = 600,  SERVO4_US_MAX = 2400;
static int SERVO5_US_MIN = 500,  SERVO5_US_MAX = 2500; // Wider for more range

// ===== Servo setup =====
Servo servo1, servo2, servo3, servo4, servo5;
const uint8_t SERVO1_PIN = 3;
const uint8_t SERVO2_PIN = 5;
const uint8_t SERVO3_PIN = 6;
const uint8_t SERVO4_PIN = 9;
const uint8_t SERVO5_PIN = 10;

// Logical angles (0–270)
int pos1 = 135, pos2 = 135, pos3 = 135, pos4 = 135, pos5 = 135;
int target1 = 135, target2 = 135, target3 = 135, target4 = 135, target5 = 135;

bool allForward = false;
bool allReverse = false;

// ===== Independent speeds =====
// Group for servos 1–4
unsigned long lastMoveTime14 = 0;
unsigned long MOVE_INTERVAL_14 = 25;  // ms (was 6)
int           STEP_SIZE_14     = 2;  // deg per tick (was 4)

// Servo 5
unsigned long lastMoveTime5 = 0;
unsigned long MOVE_INTERVAL_5 = 2;   // ms (start same as before; tune as you like)
int           STEP_SIZE_5     = 18;   // deg per tick

// ===== Helpers =====
static inline int clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

int angleToPulse(int angle, int usMin, int usMax) {
  angle = clamp(angle, 0, 270);
  return map(angle, 0, 270, usMin, usMax);
}

// Step toward a target by a configurable step size
int stepTowardBy(int current, int target, int step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

void attachAll() {
  if (!servo1.attached()) servo1.attach(SERVO1_PIN, SERVO1_US_MIN, SERVO1_US_MAX);
  if (!servo2.attached()) servo2.attach(SERVO2_PIN, SERVO2_US_MIN, SERVO2_US_MAX);
  if (!servo3.attached()) servo3.attach(SERVO3_PIN, SERVO3_US_MIN, SERVO3_US_MAX);
  if (!servo4.attached()) servo4.attach(SERVO4_PIN, SERVO4_US_MIN, SERVO4_US_MAX);
  if (!servo5.attached()) servo5.attach(SERVO5_PIN, SERVO5_US_MIN, SERVO5_US_MAX);
}

void detachAll() {
  if (servo1.attached()) servo1.detach();
  if (servo2.attached()) servo2.detach();
  if (servo3.attached()) servo3.detach();
  if (servo4.attached()) servo4.detach();
  if (servo5.attached()) servo5.detach();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Stabilize signal pins before attaching
  pinMode(SERVO1_PIN, OUTPUT); digitalWrite(SERVO1_PIN, LOW);
  pinMode(SERVO2_PIN, OUTPUT); digitalWrite(SERVO2_PIN, LOW);
  pinMode(SERVO3_PIN, OUTPUT); digitalWrite(SERVO3_PIN, LOW);
  pinMode(SERVO4_PIN, OUTPUT); digitalWrite(SERVO4_PIN, LOW);
  pinMode(SERVO5_PIN, OUTPUT); digitalWrite(SERVO5_PIN, LOW);
  delay(100);

  attachAll();

  // Initialize positions
  servo1.writeMicroseconds(angleToPulse(pos1, SERVO1_US_MIN, SERVO1_US_MAX));
  servo2.writeMicroseconds(angleToPulse(pos2, SERVO2_US_MIN, SERVO2_US_MAX));
  servo3.writeMicroseconds(angleToPulse(pos3, SERVO3_US_MIN, SERVO3_US_MAX));
  servo4.writeMicroseconds(angleToPulse(pos4, SERVO4_US_MIN, SERVO4_US_MAX));
  servo5.writeMicroseconds(angleToPulse(pos5, SERVO5_US_MIN, SERVO5_US_MAX));

  Serial.println("Servo controller ready.");
  Serial.println("Tips:");
  Serial.println("  • SET:a1,a2,a3,a4,a5        (angles 0–270)");
  Serial.println("  • ALLFWD / ALLREV / STOP");
  Serial.println("  • S5SET:angle               (0–270)");
  Serial.println("  • S5US:micros               (direct drive 500–2500 within S5 limits)");
  Serial.println("  • S5LIMIT:min,max           (µs window for #5)");
  Serial.println("  • SPD14:step,interval_ms    (speed for servos 1–4)");
  Serial.println("  • SPD5:step,interval_ms     (speed for servo 5)");
}

void loop() {
  // 1) Handle incoming command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("SET:")) {
      int a1, a2, a3, a4, a5;
      if (sscanf(line.c_str() + 4, "%d,%d,%d,%d,%d", &a1, &a2, &a3, &a4, &a5) == 5) {
        attachAll();
        target1 = clamp(a1, 0, 270);
        target2 = clamp(a2, 0, 270);
        target3 = clamp(a3, 0, 270);
        target4 = clamp(a4, 0, 270);
        target5 = clamp(a5, 0, 270);
        allForward = allReverse = false;
      }
    } else if (line == "ALLFWD") {
      allForward = true;  allReverse = false;
    } else if (line == "ALLREV") {
      allForward = false; allReverse = true;
    } else if (line == "STOP") {
      allForward = allReverse = false;
      target1 = pos1; target2 = pos2; target3 = pos3; target4 = pos4; target5 = pos5;
      detachAll();
    }
    // --- Servo 5 specific helpers ---
    else if (line.startsWith("S5SET:")) {
      int a;
      if (sscanf(line.c_str() + 6, "%d", &a) == 1) {
        attachAll();
        target5 = clamp(a, 0, 270);
        allForward = allReverse = false;
        Serial.print("S5 target angle = "); Serial.println(target5);
      }
    } else if (line.startsWith("S5US:")) {
      int us;
      if (sscanf(line.c_str() + 5, "%d", &us) == 1) {
        us = clamp(us, SERVO5_US_MIN, SERVO5_US_MAX);
        if (!servo5.attached()) servo5.attach(SERVO5_PIN, SERVO5_US_MIN, SERVO5_US_MAX);
        servo5.writeMicroseconds(us);
        int a = map(us, SERVO5_US_MIN, SERVO5_US_MAX, 0, 270);
        pos5 = target5 = clamp(a, 0, 270);
        Serial.print("S5 direct µs = "); Serial.println(us);
      }
    } else if (line.startsWith("S5LIMIT:")) {
      int mn, mx;
      if (sscanf(line.c_str() + 8, "%d,%d", &mn, &mx) == 2) {
        mn = clamp(mn, 400, 1200);
        mx = clamp(mx, 1800, 2800);
        if (mx - mn < 600) {
          Serial.println("Rejected: window too narrow.");
        } else {
          SERVO5_US_MIN = mn; SERVO5_US_MAX = mx;
          if (servo5.attached()) servo5.detach();
          servo5.attach(SERVO5_PIN, SERVO5_US_MIN, SERVO5_US_MAX);
          servo5.writeMicroseconds(angleToPulse(pos5, SERVO5_US_MIN, SERVO5_US_MAX));
          Serial.print("S5 limits set to ");
          Serial.print(SERVO5_US_MIN); Serial.print("–"); Serial.println(SERVO5_US_MAX);
        }
      }
    }
    // --- New: runtime speed tuning ---
    else if (line.startsWith("SPD14:")) {
      int step, interval;
      if (sscanf(line.c_str() + 6, "%d,%d", &step, &interval) == 2) {
        STEP_SIZE_14     = clamp(step, 1, 30);
        MOVE_INTERVAL_14 = (unsigned long) clamp(interval, 1, 100);
        Serial.print("Servos 1–4 speed -> step=");
        Serial.print(STEP_SIZE_14);
        Serial.print(", interval(ms)=");
        Serial.println(MOVE_INTERVAL_14);
      }
    } else if (line.startsWith("SPD5:")) {
      int step, interval;
      if (sscanf(line.c_str() + 5, "%d,%d", &step, &interval) == 2) {
        STEP_SIZE_5     = clamp(step, 1, 30);
        MOVE_INTERVAL_5 = (unsigned long) clamp(interval, 1, 100);
        Serial.print("Servo 5 speed -> step=");
        Serial.print(STEP_SIZE_5);
        Serial.print(", interval(ms)=");
        Serial.println(MOVE_INTERVAL_5);
      }
    }
  }

  // 2) Update targets if in ALLFWD or ALLREV (targets only; speed is applied in stepping)
  if (allForward) {
    target1 = clamp(target1 + STEP_SIZE_14, 0, 270);
    target2 = clamp(target2 + STEP_SIZE_14, 0, 270);
    target3 = clamp(target3 + STEP_SIZE_14, 0, 270);
    target4 = clamp(target4 + STEP_SIZE_14, 0, 270);
  } else if (allReverse) {
    target1 = clamp(target1 - STEP_SIZE_14, 0, 270);
    target2 = clamp(target2 - STEP_SIZE_14, 0, 270);
    target3 = clamp(target3 - STEP_SIZE_14, 0, 270);
    target4 = clamp(target4 - STEP_SIZE_14, 0, 270);
  }

  const unsigned long now = millis();

  // 3a) Smooth movement for servos 1–4 (own timer & speed)
  if (now - lastMoveTime14 >= MOVE_INTERVAL_14) {
    pos1 = stepTowardBy(pos1, target1, STEP_SIZE_14);
    pos2 = stepTowardBy(pos2, target2, STEP_SIZE_14);
    pos3 = stepTowardBy(pos3, target3, STEP_SIZE_14);
    pos4 = stepTowardBy(pos4, target4, STEP_SIZE_14);

    if (servo1.attached()) servo1.writeMicroseconds(angleToPulse(pos1, SERVO1_US_MIN, SERVO1_US_MAX));
    if (servo2.attached()) servo2.writeMicroseconds(angleToPulse(pos2, SERVO2_US_MIN, SERVO2_US_MAX));
    if (servo3.attached()) servo3.writeMicroseconds(angleToPulse(pos3, SERVO3_US_MIN, SERVO3_US_MAX));
    if (servo4.attached()) servo4.writeMicroseconds(angleToPulse(pos4, SERVO4_US_MIN, SERVO4_US_MAX));

    lastMoveTime14 = now;
  }

  // 3b) Smooth movement for servo 5 (own timer & speed)
  if (now - lastMoveTime5 >= MOVE_INTERVAL_5) {
    pos5 = stepTowardBy(pos5, target5, STEP_SIZE_5);
    if (servo5.attached()) servo5.writeMicroseconds(angleToPulse(pos5, SERVO5_US_MIN, SERVO5_US_MAX));
    lastMoveTime5 = now;
  }
}
