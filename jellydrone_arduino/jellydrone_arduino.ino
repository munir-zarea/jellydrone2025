#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// Servo setup
Servo servo1, servo2, servo3, servo4, servo5;
const uint8_t SERVO1_PIN = 3;
const uint8_t SERVO2_PIN = 5;
const uint8_t SERVO3_PIN = 6;
const uint8_t SERVO4_PIN = 9;
const uint8_t SERVO5_PIN = 10;

int pos1 = 135, pos2 = 135, pos3 = 135, pos4 = 135, pos5 = 135;
int target1 = 135, target2 = 135, target3 = 135, target4 = 135, target5 = 135;

bool allForward = false;
bool allReverse = false;

unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 20;  // ms
const int STEP_SIZE = 1;

// IMU
MPU6050 imu;
unsigned long lastIMUTime = 0;
const unsigned long IMU_INTERVAL = 100; // 10 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Stabilize signal pins before attaching
  pinMode(SERVO1_PIN, OUTPUT); digitalWrite(SERVO1_PIN, LOW);
  pinMode(SERVO2_PIN, OUTPUT); digitalWrite(SERVO2_PIN, LOW);
  pinMode(SERVO3_PIN, OUTPUT); digitalWrite(SERVO3_PIN, LOW);
  pinMode(SERVO4_PIN, OUTPUT); digitalWrite(SERVO4_PIN, LOW);
  pinMode(SERVO5_PIN, OUTPUT); digitalWrite(SERVO5_PIN, LOW);
  delay(100);

  // Attach and write initial positions
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  servo1.writeMicroseconds(angleToWidePulse(pos1));
  servo2.writeMicroseconds(angleToWidePulse(pos2));
  servo3.writeMicroseconds(angleToWidePulse(pos3));
  servo4.writeMicroseconds(angleToWidePulse(pos4));
  servo5.writeMicroseconds(angleToWidePulse(pos5));

  // IMU initialization
  Wire.begin();
  imu.initialize();

  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 ready");
  }

  Serial.println("Servo controller ready.");
}


void loop() {
  // 1) Handle incoming command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("SET:")) {
      int a1, a2, a3, a4, a5;
      if (sscanf(line.c_str() + 4, "%d,%d,%d,%d,%d", &a1, &a2, &a3, &a4, &a5) == 5) {
        if (!servo1.attached()) servo1.attach(SERVO1_PIN);
        if (!servo2.attached()) servo2.attach(SERVO2_PIN);
        if (!servo3.attached()) servo3.attach(SERVO3_PIN);
        if (!servo4.attached()) servo4.attach(SERVO4_PIN);
        if (!servo5.attached()) servo5.attach(SERVO5_PIN);

        target1 = constrain(a1, 0, 270);
        target2 = constrain(a2, 0, 270);
        target3 = constrain(a3, 0, 270);
        target4 = constrain(a4, 0, 270);
        target5 = constrain(a5, 0, 270);
        allForward = allReverse = false;
      }
    } else if (line == "ALLFWD") {
      allForward = true;
      allReverse = false;
    } else if (line == "ALLREV") {
      allForward = false;
      allReverse = true;
    } else if (line == "STOP") {
      allForward = false;
      allReverse = false;
      target1 = pos1;
      target2 = pos2;
      target3 = pos3;
      target4 = pos4;
      target5 = pos5;

      servo1.detach();
      servo2.detach();
      servo3.detach();
      servo4.detach();
      servo5.detach();
    }
  }

  // 2) Update targets if in ALLFWD or ALLREV
  if (allForward) {
    target1 = constrain(target1 + STEP_SIZE, 0, 270);
    target2 = constrain(target2 + STEP_SIZE, 0, 270);
    target3 = constrain(target3 + STEP_SIZE, 0, 270);
    target4 = constrain(target4 + STEP_SIZE, 0, 270);
  } else if (allReverse) {
    target1 = constrain(target1 - STEP_SIZE, 0, 270);
    target2 = constrain(target2 - STEP_SIZE, 0, 270);
    target3 = constrain(target3 - STEP_SIZE, 0, 270);
    target4 = constrain(target4 - STEP_SIZE, 0, 270);
  }

  // 3) Smooth movement
  unsigned long now = millis();
  if (now - lastMoveTime >= MOVE_INTERVAL) {
    pos1 = stepToward(pos1, target1);
    pos2 = stepToward(pos2, target2);
    pos3 = stepToward(pos3, target3);
    pos4 = stepToward(pos4, target4);
    pos5 = stepToward(pos5, target5);

    if (servo1.attached()) servo1.writeMicroseconds(angleToWidePulse(pos1));
    if (servo2.attached()) servo2.writeMicroseconds(angleToWidePulse(pos2));
    if (servo3.attached()) servo3.writeMicroseconds(angleToWidePulse(pos3));
    if (servo4.attached()) servo4.writeMicroseconds(angleToWidePulse(pos4));
    if (servo5.attached()) servo5.writeMicroseconds(angleToWidePulse(pos5));

    lastMoveTime = now;
  }

  // 4) Read and publish IMU
  if (now - lastIMUTime >= IMU_INTERVAL) {
    publishIMU();
    lastIMUTime = now;
  }
}

int stepToward(int current, int target) {
  if (current < target) return min(current + STEP_SIZE, target);
  if (current > target) return max(current - STEP_SIZE, target);
  return current;
}

// Maps logical 0–180 to 600–2400 µs to unlock more servo range
int angleToWidePulse(int angle) {
  angle = constrain(angle, 0, 270);
  return map(angle, 0, 270, 600, 2400);
}

void publishIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_scale = 9.81 / 16384.0;
  float gyro_scale = 3.14159 / (180.0 * 131.0);

  float ax_mps2 = ax * accel_scale;
  float ay_mps2 = ay * accel_scale;
  float az_mps2 = az * accel_scale;

  float gx_rads = gx * gyro_scale;
  float gy_rads = gy * gyro_scale;
  float gz_rads = gz * gyro_scale;

  Serial.print("IMU:");
  Serial.print(ax_mps2, 3); Serial.print(',');
  Serial.print(ay_mps2, 3); Serial.print(',');
  Serial.print(az_mps2, 3); Serial.print(',');
  Serial.print(gx_rads, 3); Serial.print(',');
  Serial.print(gy_rads, 3); Serial.print(',');
  Serial.println(gz_rads, 3);

  delay(10);
}
