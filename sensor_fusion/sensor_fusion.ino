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


struct raw_data {   // Raw data
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
} rd;

struct {    // Raw angle data
  float accel_roll, accel_pitch, mag_yaw;
  float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
} rad;

Adafruit_BNO08x imu(IMU1_RESET);
sh2_SensorValue_t sensorValue;

long reportInterval = 100000;
unsigned long lastTime;

 float P[3][3] = {  // Covariance matrix (3x3)
  {0.1, 0, 0},
  {0, 0.1, 0},
  {0, 0, 0.1}
 };

const float Q[3][3] = {   // Process noise covariance
  {0.001, 0, 0},
  {0, 0.001, 0},
  {0, 0, 0.001}
};

const float R[2][2] = {
  {0.03, 0},
  {0, 0.03}
};


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
  digitalWrite(BOOTN, HIGH);

  if (!imu.begin_SPI(IMU1_CS, IMU1_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
  
  lastTime = micros();
}


float wrap_angle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}


void ekf_update(rad angle_data) {
  float  A[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  }  

  P


}


void loop() {
  if (imu.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER: {
        // unsigned long currentTime = micros();
        // float dt = (currentTime - lastTime) / 1000000.0;
        // lastTime = currentTime;
        rd.accel_x = sensorValue.un.accelerometer.x;
        rd.accel_y = sensorValue.un.accelerometer.y;
        rd.accel_z = sensorValue.un.accelerometer.z;

        rad.accel_roll = atan2f(rd.accel_y, rd.accel_z) * RAD_TO_DEG;
        rad.accel_pitch = atan2f(-rd.accel_x, sqrtf(rd.accel_y * rd.accel_y + rd.accel_z * rd.accel_z)) * RAD_TO_DEG;

        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        rd.gyro_x = sensorValue.un.gyroscope.x;
        rd.gyro_y = sensorValue.un.gyroscope.y;
        rd.gyro_z = sensorValue.un.gyroscope.z;

        // rad.gyro_roll_rate = rd.gyro_x + rd.gyro_y*sinf(rad.gyro_roll)*tanf(rad.gyro_pitch) + rd.gyro_z*cosf(rad.gyro_roll)*tanf(rad.gyro_pitch);
        // rad.gyro_pitch_rate = rd.gyro_y*cosf(rad.gyro_roll) - rd.gyro_z*sinf(rad.gyro_roll);
        // rad.gyro_yaw_rate = rd.gyro_y*(sinf(rad.gyro_roll)/cosf(rad.gyro_pitch)) + rd.gyro_z*(cosf(rad.gyro_roll)/cosf(rad.gyro_pitch));

        // rad.gyro_roll += rad.gyro_roll_rate * dt * RAD_TO_DEG;
        // rad.gyro_pitch += rad.gyro_pitch_rate * dt * RAD_TO_DEG;
        // rad.gyro_yaw += rad.gyro_yaw_rate * dt * RAD_TO_DEG;

        rad.gyro_roll += rd.gyro_x * dt * RAD_TO_DEG;
        rad.gyro_pitch += rd.gyro_y * dt * RAD_TO_DEG;
        rad.gyro_yaw += rd.gyro_z * dt * RAD_TO_DEG;

        rad.gyro_roll = wrap_angle(rad.gyro_roll);
        rad.gyro_pitch = wrap_angle(rad.gyro_pitch);
        rad.gyro_yaw = wrap_angle(rad.gyro_yaw);

        break;
      }
      case SH2_MAGNETIC_FIELD_CALIBRATED: {
        rd.mag_x = sensorValue.un.magneticField.x;
        rd.mag_y = sensorValue.un.magneticField.y;
        rd.mag_z = sensorValue.un.magneticField.z;

        rad.mag_yaw = atan2f(rd.mag_y, rd.mag_x) * RAD_TO_DEG;

        break;
      }
    }

    // RAW DATA
    
    Serial.print(millis()/1000); Serial.print("\t");
    Serial.print(rd.accel_roll); Serial.print("\t");

    Serial.print(rd.accel_pitch); Serial.print("\t");
    Serial.println(rd.mag_yaw);

    // SENSOR FUSION DATA (APPLIED EKF)
    ekf_update(rad);

    // Serial.print(rad.gyro_roll); Serial.print("\t");
    // Serial.print(rad.gyro_pitch); Serial.print("\t");
    // Serial.println(rad.gyro_yaw); 
    }
    
    delay(10);
}

