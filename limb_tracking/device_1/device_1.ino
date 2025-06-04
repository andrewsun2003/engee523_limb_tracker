#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include "AsyncUDP.h"

#define IMU1_CS 34
#define IMU1_INT 2
#define IMU1_RESET 5
#define PS0 42
#define PS1 41
#define BOOTN 1

const char *ssid = "Limb Tracker";
const char *password = "engee523";

AsyncUDP udp;
Adafruit_BNO08x imu(IMU1_RESET);
sh2_SensorValue_t sensorValue;

long reportInterval = 10000;  // 100Hz

void setReports() {
  Serial.println("Enabling quaternion output...");
  if (!imu.enableReport(SH2_ROTATION_VECTOR, reportInterval)) {
    Serial.println("Could not enable rotation vector");
    while (1) delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  if (udp.listen(1234)) {
    Serial.println("UDP listening on port 1234...");
  }

  pinMode(PS0, OUTPUT); digitalWrite(PS0, HIGH);
  pinMode(PS1, OUTPUT); digitalWrite(PS1, HIGH);
  pinMode(BOOTN, OUTPUT); digitalWrite(BOOTN, HIGH);

  if (!imu.begin_SPI(IMU1_CS, IMU1_INT)) {
    Serial.println("Failed to find BNO08x");
    while (1) delay(10);
  }

  Serial.println("BNO08x Found!");
  setReports();
}

void loop() {
  if (imu.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float real = sensorValue.un.rotationVector.real;
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;

      char quat_msg[100];
      snprintf(quat_msg, sizeof(quat_msg), "IMU 1 Quaternion: %.4f, %.4f, %.4f, %.4f", real, i, j, k);
      udp.broadcastTo(quat_msg, 1234);
    }
  }

  delay(10);
}
