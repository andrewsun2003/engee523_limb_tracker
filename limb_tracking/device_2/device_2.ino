#include <stdio.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include "AsyncUDP.h"


#define IMU1_CS 4
#define IMU1_INT 44
#define IMU1_RESET 43
#define PS0 6
#define PS1 5
#define BOOTN 3

const char *ssid = "Limb Tracker";
const char *password = "engee523";

AsyncUDP udp;
Adafruit_BNO08x imu(IMU1_RESET);
sh2_SensorValue_t sensorValue;

long reportInterval = 10000;

void setReports() {
  Serial.println("Enabling quaternion output...");
  if (!imu.enableReport(SH2_ROTATION_VECTOR, reportInterval)) {
    Serial.println("Could not enable rotation vector");
    while (1) delay(10);
  }
}

void setup(void) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Attempting to connect to WiFi...");

  // Keep trying until it connects
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(PS0, OUTPUT); digitalWrite(PS0, HIGH);
  pinMode(PS1, OUTPUT); digitalWrite(PS1, HIGH);
  pinMode(BOOTN, OUTPUT); digitalWrite(BOOTN, HIGH);

  if (!imu.begin_SPI(IMU1_CS, IMU1_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
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
      // snprintf(quat_msg, sizeof(quat_msg), "IMU 2 Quaternion: %.4f, %.4f, %.4f, %.4f", real, i, j, k);
      snprintf(quat_msg, sizeof(quat_msg), "IMU 3 Quaternion: %.4f, %.4f, %.4f, %.4f", real, i, j, k);

      udp.broadcastTo(quat_msg, 1234);
    }
  }

  delay(10);
}
