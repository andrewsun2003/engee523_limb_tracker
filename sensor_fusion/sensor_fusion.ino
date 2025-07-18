#include <stdio.h>
#include <math.h>
#include <Adafruit_BNO08x.h>


#define IMU1_CS 4
#define IMU1_INT 44
#define IMU1_RESET 43

#define RAD_TO_DEG 57.2958

bool offset_flag = true;
bool mag_yaw_received = false;
bool dmp_yaw_received = false;

float mag_yaw_offset = 0;

const float alpha = 0.05; 

float gyro_filtered[3] = {0.0, 0.0, 0.0};

struct raw_data {   // Raw data
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
} rd;

struct raw_angle_data {    // Raw angle data
  float accel_roll, accel_pitch, mag_yaw;
} rad;

float x[3] = {0, 0, 0};   // State estimate

float g[3] = {0, 0, 0};   // State estimate

float x_prev[3] = {0, 0, 0};   // State estimate

 float P[3][3] = {  // Covariance matrix (3x3)
  {0.01, 0, 0},
  {0, 0.01, 0},
  {0, 0, 0.01}
 };

float  A[3][3] = {  // System behaviour in isolation
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};  

const float Q[3][3] = {   // Process noise covariance
  {0.001, 0, 0},
  {0, 0.001, 0},
  {0, 0, 0.001}
};

const float R[3][3] = {   // Measurement noise covariance
  {0.03, 0, 0},
  {0, 0.03, 0},
  {0, 0, 0.05}
};

const float I[3][3] = {   // Identity matrix
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x imu(IMU1_RESET);
sh2_SensorValue_t sensorValue;

long reportInterval = 100000;
unsigned long lastTime;


void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!imu.begin_SPI(IMU1_CS, IMU1_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
  
  lastTime = micros();
}


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
  if (!imu.enableReport(SH2_GYRO_INTEGRATED_RV, reportInterval)) {
    Serial.println("Could not enable stabilized remote vector");
    while (1) { delay(10); }
  }
}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


float wrap_angle(float angle) {
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
  return angle;
}


void multiply3x3(const float A[3][3], const float B[3][3], float result[3][3]) {
    // Initialize result matrix to zero
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result[i][j] = 0.0f;

    // Perform matrix multiplication
    for (int i = 0; i < 3; i++) {         // row of A
        for (int j = 0; j < 3; j++) {     // column of B
            for (int k = 0; k < 3; k++) { // shared dimension
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


void invert3x3(const float in[3][3], float out[3][3]) {
  float det =
    in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1]) -
    in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
    in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);

  float invDet = 1.0f / det;

  out[0][0] =  (in[1][1]*in[2][2] - in[1][2]*in[2][1]) * invDet;
  out[0][1] = -(in[0][1]*in[2][2] - in[0][2]*in[2][1]) * invDet;
  out[0][2] =  (in[0][1]*in[1][2] - in[0][2]*in[1][1]) * invDet;

  out[1][0] = -(in[1][0]*in[2][2] - in[1][2]*in[2][0]) * invDet;
  out[1][1] =  (in[0][0]*in[2][2] - in[0][2]*in[2][0]) * invDet;
  out[1][2] = -(in[0][0]*in[1][2] - in[0][2]*in[1][0]) * invDet;

  out[2][0] =  (in[1][0]*in[2][1] - in[1][1]*in[2][0]) * invDet;
  out[2][1] = -(in[0][0]*in[2][1] - in[0][1]*in[2][0]) * invDet;
  out[2][2] =  (in[0][0]*in[1][1] - in[0][1]*in[1][0]) * invDet;
}


void copy3x3(float src[3][3], float dest[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      dest[i][j] = src[i][j];
}


void ekf_update(struct raw_angle_data *rad, float g[3], float x[3], float x_prev[3], float P[3][3], const float A[3][3], const float Q[3][3], const float R[3][3], const float I[3][3]) { 
  
  // State
  for (int i = 0; i < 3; i++) {
      x[i] = wrap_angle(x_prev[i] + g[i]);
  }

  // State covariance: P = A*P*A^T + Q (A = identity matrix)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] += Q[i][j];
    }
  }

  float z[3] = {rad->accel_roll, rad->accel_pitch, rad->mag_yaw};

  // Measurement residual: y = z - Hx
  float y[3];
  for (int i = 0; i < 3; i++) {
    y[i] = wrap_angle(z[i] - x[i]);
  }

  // Innovation covariance: S = H*P*H^T + R
  float S[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      S[i][j] = P[i][j] + R[i][j];
    }
  }

  // Kalman gain: K = P*H^T*S^-1
  float S_inv[3][3];

  invert3x3(S, S_inv);
  
  float K[3][3];

  multiply3x3(P, S_inv, K);

  // Update state: x = x + K*y
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      x[i] += K[i][j] * y[j];
    }
    x[i] = wrap_angle(x[i]);
  }

  // Update state covariance: P = (I - KH)*P
  float I_minus_k[3][3];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        I_minus_k[i][j] = I[i][j] - K[i][j];
      }
  }

  float temp[3][3];
  multiply3x3(I_minus_k, P, temp);
  copy3x3(temp, P);

  for (int i = 0; i < 3; i++) {
  x_prev[i] = x[i];
  } 
}


void loop() {
  if (imu.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER: {
        rd.accel_x = sensorValue.un.accelerometer.x;
        rd.accel_y = sensorValue.un.accelerometer.y;
        rd.accel_z = sensorValue.un.accelerometer.z;

        rad.accel_roll = atan2f(rd.accel_y, rd.accel_z);
        rad.accel_pitch = atan2f(-rd.accel_x, sqrtf(rd.accel_y * rd.accel_y + rd.accel_z * rd.accel_z));

        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        rd.gyro_x = sensorValue.un.gyroscope.x;
        rd.gyro_y = sensorValue.un.gyroscope.y;
        rd.gyro_z = sensorValue.un.gyroscope.z;

        // Apply low-pass filter to raw gyro data (in rad/s)
        gyro_filtered[0] = alpha * rd.gyro_x + (1 - alpha) * gyro_filtered[0];
        gyro_filtered[1] = alpha * rd.gyro_y + (1 - alpha) * gyro_filtered[1];
        gyro_filtered[2] = alpha * rd.gyro_z + (1 - alpha) * gyro_filtered[2];

        // Integrate filtered gyro data (convert to degrees)
        g[0] = gyro_filtered[0] * dt;
        g[1] = gyro_filtered[1] * dt;
        g[2] = gyro_filtered[2] * dt;

        g[0] = wrap_angle(g[0]);
        g[1] = wrap_angle(g[1]);
        g[2] = wrap_angle(g[2]);

        break;
      }
      case SH2_MAGNETIC_FIELD_CALIBRATED: {
        mag_yaw_received = true;

        rd.mag_x = sensorValue.un.magneticField.x;
        rd.mag_y = sensorValue.un.magneticField.y;
        rd.mag_z = sensorValue.un.magneticField.z;

        float roll = rad.accel_roll;
        float pitch = rad.accel_pitch;

        // Tilt-compensated magnetic field
        float Bx = rd.mag_x * cos(pitch) + rd.mag_z * sin(pitch);
        float By = rd.mag_x * sin(roll) * sin(pitch) + rd.mag_y * cos(roll) - rd.mag_z * sin(roll) * cos(pitch);

        rad.mag_yaw = atan2f(-By, Bx);
        // rad.mag_yaw = atan2f(rd.mag_y, rd.mag_x) * RAD_TO_DEG;
        rad.mag_yaw = wrap_angle(rad.mag_yaw);

        if (offset_flag == false) {
          rad.mag_yaw -= mag_yaw_offset;
          rad.mag_yaw = wrap_angle(rad.mag_yaw);
        }

        break;
      }
      case SH2_GYRO_INTEGRATED_RV:
        dmp_yaw_received = true;
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);

        break;
    }

    if (offset_flag == true && mag_yaw_received && dmp_yaw_received) {
      mag_yaw_offset = wrap_angle(rad.mag_yaw - ypr.yaw);
      offset_flag = false;  // Only do this once
    }

    if (offset_flag == false) {
      ekf_update(&rad, g, x, x_prev, P, A, Q, R, I);
      
      Serial.print(millis()/1000); Serial.print("\t");

      // RAW DATA
      Serial.print(rd.accel_x); Serial.print("\t");
      Serial.print(rd.accel_y); Serial.print("\t");
      Serial.print(rd.accel_z); Serial.print("\t");
      
      Serial.print(rd.gyro_x); Serial.print("\t");
      Serial.print(rd.gyro_y); Serial.print("\t");
      Serial.print(rd.gyro_z); Serial.print("\t");

      Serial.print(rd.mag_x); Serial.print("\t");
      Serial.print(rd.mag_y); Serial.print("\t");
      Serial.print(rd.mag_z); Serial.print("\t");

      Serial.print(rad.accel_roll * RAD_TO_DEG); Serial.print("\t");
      Serial.print(rad.accel_pitch * RAD_TO_DEG); Serial.print("\t");
      Serial.print(rad.mag_yaw * RAD_TO_DEG); Serial.print("\t");

      // SENSOR FUSION DATA (APPLIED EKF)
      Serial.print(x[0] * RAD_TO_DEG); Serial.print("\t");
      Serial.print(x[1] * RAD_TO_DEG); Serial.print("\t");
      Serial.print(x[2] * RAD_TO_DEG); Serial.print("\t");

      // DMP DATA 
      Serial.print(ypr.roll); Serial.print("\t");
      Serial.print(ypr.pitch);  Serial.print("\t");
      Serial.println(ypr.yaw);
    }
  }
    delay(10);
}
