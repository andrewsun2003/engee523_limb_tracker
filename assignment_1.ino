#include <stdio.h>
#include <math.h>
#include <Adafruit_BNO08x.h>


#define IMU1_CS 34
#define IMU1_INT 2
#define IMU1_RESET 5

#define PS0 42
#define PS1 41

#define BOOTN 1

#define RAD_TO_DEG 57.2958


struct raw_data {
  float accel_x, accel_y, accel_z;
  float accel_roll, accel_pitch;

  float gyro_x, gyro_y, gyro_z;
  float gyro_roll, gyro_pitch, gyro_yaw = 0.0;

  float mag_x, mag_y, mag_z;
} rd;

Adafruit_BNO08x imu(IMU1_RESET);
sh2_SensorValue_t sensorValue;

long reportInterval = 100000;
unsigned long lastTime;


void setReports() {
  Serial.println("Setting desired reports");

  if (! imu.enableReport(SH2_ACCELEROMETER, reportInterval)) {
    Serial.println("Could not enable accelerometer");
    while (1) { delay(10); }
  }
  if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, reportInterval)) {
    Serial.println("Could not enable gyroscope");
    while (1) { delay(10); }
  }
  if (!imu.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, reportInterval)) {
    Serial.println("Could not enable magnetometer");
    while (1) { delay(10); }
  }
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(PS0, OUTPUT);
  digitalWrite(PS0, HIGH);

  pinMode(PS1, OUTPUT);
  digitalWrite(PS1, HIGH);

  pinMode(BOOTN, OUTPUT);
  digitalWrite(BOOTN, LOW);

  if (!imu.begin_SPI(IMU1_CS, IMU1_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
  
  lastTime = micros();
}


void loop() {
  if (imu.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER: 
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;
        rd.accel_x = sensorValue.un.accelerometer.x;
        rd.accel_y = sensorValue.un.accelerometer.y;
        rd.accel_z = sensorValue.un.accelerometer.z;
        rd.accel_roll = atan2f(rd.accel_y, rd.accel_z) * RAD_TO_DEG;
        rd.accel_pitch = atan2f(-rd.accel_x, sqrtf(rd.accel_y * rd.accel_y + rd.accel_z * rd.accel_z)) * RAD_TO_DEG;

        // Serial.print("ACCEL (RP): "); 
        Serial.print(millis()/1000); Serial.print("\t");
        Serial.print(rd.accel_roll); Serial.print("\t");
        Serial.println(rd.accel_pitch);

        break;
      // case SH2_GYROSCOPE_CALIBRATED: 
      //   unsigned long currentTime = micros();
      //   float dt = (currentTime - lastTime) / 1000000.0;
      //   lastTime = currentTime;

      //   rd.gyro_x = sensorValue.un.gyroscope.x;
      //   rd.gyro_y = sensorValue.un.gyroscope.y;
      //   rd.gyro_z = sensorValue.un.gyroscope.z;

      //   rd.gyro_roll += rd.gyro_x * dt * RAD_TO_DEG;
      //   rd.gyro_pitch += rd.gyro_y * dt * RAD_TO_DEG;
      //   rd.gyro_yaw += rd.gyro_z * dt * RAD_TO_DEG;

      //   Serial.print("GYRO (RPY): "); 
      //   Serial.print(rd.gyro_roll);
      //   Serial.print("\t");
      //   Serial.print(rd.gyro_pitch);
      //   Serial.print("\t");
      //   Serial.println(rd.gyro_yaw);

      //   break;
      // case SH2_MAGNETIC_FIELD_CALIBRATED:
        // rd.mag_x = sensorValue.un.magneticField.x;
        // rd.mag_y = sensorValue.un.magneticField.y;
        // rd.mag_z = sensorValue.un.magneticField.z;

        // Serial.print("MAG: ");
        // Serial.print(rd.mag_x);
        // Serial.print("\t");
        // Serial.print(rd.mag_y);
        // Serial.print("\t");
        // Serial.println(rd.mag_z);

        // break;
      }
    }
    delay(10);
}