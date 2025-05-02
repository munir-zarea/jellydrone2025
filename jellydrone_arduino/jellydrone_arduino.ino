#include <Servo.h>

// Servo setup
Servo servo1, servo2, servo3, servo4, servo5;
const uint8_t SERVO1_PIN = 3;
const uint8_t SERVO2_PIN = 5;
const uint8_t SERVO3_PIN = 6;
const uint8_t SERVO4_PIN = 9;
const uint8_t SERVO5_PIN = 10;

int pos1 = 90, pos2 = 90, pos3 = 90, pos4 = 90, pos5 = 90;
int target1 = 90, target2 = 90, target3 = 90, target4 = 90, target5 = 90;

bool allForward = false;
bool allReverse = false;

unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 20;  // ms
const int STEP_SIZE = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
  servo5.write(pos5);

  Serial.println("Servo controller ready.");
}

void loop() {
  // 1) Read incoming command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("SET:")) {
      int a1,a2,a3,a4,a5;
      if (sscanf(line.c_str()+4, "%d,%d,%d,%d,%d",
                 &a1,&a2,&a3,&a4,&a5) == 5) {
        target1 = constrain(a1,0,180);
        target2 = constrain(a2,0,180);
        target3 = constrain(a3,0,180);
        target4 = constrain(a4,0,180);
        target5 = constrain(a5,0,180);
        allForward = allReverse = false;
      }
    }
    else if (line == "ALLFWD") {
      allForward = true;
      allReverse = false;
    }
    else if (line == "ALLREV") {
      allForward = false;
      allReverse = true;
    }
    else if (line == "STOP") {
      // clear drive flags
      allForward = false;
      allReverse = false;
      // freeze targets here:
      target1 = pos1;
      target2 = pos2;
      target3 = pos3;
      target4 = pos4;
      target5 = pos5;
    }
  }

  // 2) If still in ALLFWD/ALLREV, update targets
  if (allForward) {
    target1 = constrain(target1 + STEP_SIZE, 0, 180);
    target2 = constrain(target2 + STEP_SIZE, 0, 180);
    target3 = constrain(target3 + STEP_SIZE, 0, 180);
    target4 = constrain(target4 + STEP_SIZE, 0, 180);
  }
  else if (allReverse) {
    target1 = constrain(target1 - STEP_SIZE, 0, 180);
    target2 = constrain(target2 - STEP_SIZE, 0, 180);
    target3 = constrain(target3 - STEP_SIZE, 0, 180);
    target4 = constrain(target4 - STEP_SIZE, 0, 180);
  }

  // 3) Smoothly step toward targets
  unsigned long now = millis();
  if (now - lastMoveTime >= MOVE_INTERVAL) {
    pos1 = stepToward(pos1, target1);
    pos2 = stepToward(pos2, target2);
    pos3 = stepToward(pos3, target3);
    pos4 = stepToward(pos4, target4);
    pos5 = stepToward(pos5, target5);

    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);
    servo5.write(pos5);

    lastMoveTime = now;
  }
}

int stepToward(int current, int target) {
  if (current < target) return min(current + STEP_SIZE, target);
  if (current > target) return max(current - STEP_SIZE, target);
  return current;
}
