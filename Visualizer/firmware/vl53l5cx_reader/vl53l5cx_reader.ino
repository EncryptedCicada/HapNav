/*
 * VL53L5CX ToF Sensor + LSM6DSO IMU Reader for ESP32
 *
 * Reads 8x8 distance data from VL53L5CX and orientation from LSM6DSO,
 * outputs JSON over serial.
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "SparkFunLSM6DSO.h"

#define VERSION "0.1.0"

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SPEED 400000

// VL53L5CX ToF sensor
SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

// LSM6DSO IMU
LSM6DSO imu;
bool imuAvailable = false;

// Quaternion in w, x, y, z format
float quatW = 1.0f;
float quatX = 0.0f;
float quatY = 0.0f;
float quatZ = 0.0f;

// Mahony-style filter parameters
float twoKp = 2.0f * 0.8f;
float twoKi = 2.0f * 0.0f;

float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;

unsigned long lastImuMicros = 0;

void updateIMUFilter(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  // gx, gy, gz must be in rad/s
  // ax, ay, az can be in g or m/s^2, because we normalize them

  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Ignore invalid accelerometer data
  if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
    return;
  }

  // Normalize accelerometer
  recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Estimated direction of gravity
  halfvx = quatX * quatZ - quatW * quatY;
  halfvy = quatW * quatX + quatY * quatZ;
  halfvz = quatW * quatW - 0.5f + quatZ * quatZ;

  // Error is cross product between estimated and measured gravity
  halfex = (ay * halfvz - az * halfvy);
  halfey = (az * halfvx - ax * halfvz);
  halfez = (ax * halfvy - ay * halfvx);

  // Integral feedback
  if (twoKi > 0.0f) {
    integralFBx += twoKi * halfex * dt;
    integralFBy += twoKi * halfey * dt;
    integralFBz += twoKi * halfez * dt;

    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
  } else {
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Proportional feedback
  gx += twoKp * halfex;
  gy += twoKp * halfey;
  gz += twoKp * halfez;

  // Integrate quaternion rate
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);

  qa = quatW;
  qb = quatX;
  qc = quatY;

  quatW += (-qb * gx - qc * gy - quatZ * gz);
  quatX += (qa * gx + qc * gz - quatZ * gy);
  quatY += (qa * gy - qb * gz + quatZ * gx);
  quatZ += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  recipNorm = 1.0f / sqrt(quatW * quatW + quatX * quatX + quatY * quatY + quatZ * quatZ);
  quatW *= recipNorm;
  quatX *= recipNorm;
  quatY *= recipNorm;
  quatZ *= recipNorm;
}

void setup() {
  delay(2000);

  Serial.begin(115200);
  delay(1000);

  Serial.println("start");
  Serial.println("{\"status\":\"initializing\"}");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED);

  Serial.println("{\"status\":\"i2c_ready\"}");

  // Initialize VL53L5CX
  if (!sensor.begin()) {
    Serial.println("{\"error\":\"tof_sensor_init_failed\"}");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("{\"status\":\"tof_sensor_found\"}");

  sensor.setResolution(64);
  sensor.setRangingFrequency(15);
  sensor.startRanging();

  Serial.println("{\"status\":\"ranging_started\",\"resolution\":\"8x8\",\"frequency_hz\":15}");

  // Initialize LSM6DSO
  if (imu.begin()) {
    imuAvailable = true;
    Serial.println("{\"status\":\"imu_found\"}");
  } else {
    Serial.println("{\"status\":\"imu_not_found\"}");
  }

  if (imuAvailable) {
    if (imu.initialize(BASIC_SETTINGS)) {
      Serial.println("{\"status\":\"imu_settings_loaded\"}");
    } else {
      Serial.println("{\"status\":\"imu_settings_failed\"}");
    }

    lastImuMicros = micros();
    Serial.println("{\"status\":\"imu_ready\",\"mode\":\"lsm6dso_6axis_fusion\"}");
  }
}

void loop() {
  // Update IMU quaternion
  if (imuAvailable) {
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();

    // SparkFun LSM6DSO gyro is usually in deg/s.
    // Convert deg/s to rad/s for the filter.
    gx *= 0.01745329252f;
    gy *= 0.01745329252f;
    gz *= 0.01745329252f;

    unsigned long now = micros();
    float dt = (now - lastImuMicros) * 1e-6f;
    lastImuMicros = now;

    if (dt > 0.0f && dt < 0.1f) {
      updateIMUFilter(gx, gy, gz, ax, ay, az, dt);
    }
  }

  // Output ToF + quaternion JSON
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      Serial.print("{\"distances\":[");

      for (int i = 0; i < 64; i++) {
        Serial.print(measurementData.distance_mm[i]);
        if (i < 63) Serial.print(",");
      }

      Serial.print("],\"status\":[");

      for (int i = 0; i < 64; i++) {
        Serial.print(measurementData.target_status[i]);
        if (i < 63) Serial.print(",");
      }

      Serial.print("],\"quat\":[");
      Serial.print(quatW, 6); Serial.print(",");
      Serial.print(quatX, 6); Serial.print(",");
      Serial.print(quatY, 6); Serial.print(",");
      Serial.print(quatZ, 6);

      Serial.print("],\"v\":\"");
      Serial.print(VERSION);
      Serial.println("\"}");
    }
  }

  delay(1);
}