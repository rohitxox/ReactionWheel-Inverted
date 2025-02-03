#ifndef IMU_H
#define IMU_H

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// IMU Parameters
Adafruit_ICM20948 icm;
Adafruit_Sensor *accel;
Adafruit_Sensor *gyro;

// Complementary Filter Parameters
const float alpha = 0.85; // Weight for gyroscope data (0 < alpha < 1)
float rollAngle = 0.0;   
unsigned long lastTime = 0;

bool imuInit() {
  if (!icm.begin_I2C()) return false;

  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_50_4_HZ); // Optimized DLPF frequency
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS); // Set gyroscope range to 250 degrees/s

  accel = icm.getAccelerometerSensor();
  gyro = icm.getGyroSensor();
  lastTime = millis(); 
  return true;
}

float getRoll() {
  sensors_event_t a, g;
  accel->getEvent(&a);
  gyro->getEvent(&g);

  float accelRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // in seconds
  lastTime = now;

  float gyroRollRate = g.gyro.x; // in deg/s
  rollAngle = alpha * (rollAngle + gyroRollRate * dt) + (1 - alpha) * accelRoll;

  return rollAngle;
}


#endif