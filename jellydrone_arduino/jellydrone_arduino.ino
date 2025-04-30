#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// —— Configuration —— 
static const int SERVO_PINS[5] = {3, 5, 6, 9, 10};
static const int MOVE_SPEED   = 1;      // degrees per step
static const unsigned long SERVO_DT  = 20;  // ms between servo updates (~50 Hz)
static const unsigned long IMU_DT    = 5;   // ms between IMU prints (~200 Hz)

Adafruit_MPU6050 mpu;
Servo        servos[5];

// state
int          positions[5]    = {90, 90, 90, 90, 90};
int          deltas[5]       = { 0,  0,  0,  0,  0};
unsigned long last_servo_time = 0;
unsigned long last_imu_time   = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  // Attach and center servos
  for (int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(positions[i]);
  }

  // Init IMU
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Halting.");
    while (1) { delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(500);

  Serial.println("Arduino ready");
}

void loop() {
  unsigned long now = millis();

  // 1) Read any new servo commands
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int s1, s2, s3, s4, s5;
    if (sscanf(line.c_str(), "%d,%d,%d,%d,%d", &s1, &s2, &s3, &s4, &s5) == 5) {
      deltas[0] = s1;
      deltas[1] = s2;
      deltas[2] = s3;
      deltas[3] = s4;
      deltas[4] = s5;
    }
  }

  // 2) Step servos every SERVO_DT ms
  if (now - last_servo_time >= SERVO_DT) {
    for (int i = 0; i < 5; i++) {
      positions[i] = constrain(positions[i] + deltas[i] * MOVE_SPEED, 0, 180);
      servos[i].write(positions[i]);
    }
    last_servo_time = now;
  }

  // 3) Publish IMU data every IMU_DT ms
  if (now - last_imu_time >= IMU_DT) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Prefix “IMU:” so your serial_bridge picks it up
    Serial.print("IMU:");
    Serial.print(accel.acceleration.x, 2); Serial.print(',');
    Serial.print(accel.acceleration.y, 2); Serial.print(',');
    Serial.print(accel.acceleration.z, 2); Serial.print(',');
    Serial.print(gyro.gyro.x,         2); Serial.print(',');
    Serial.print(gyro.gyro.y,         2); Serial.print(',');
    Serial.println(gyro.gyro.z,       2);

    last_imu_time = now;
  }

  // tiny yield
  delay(1);
}
