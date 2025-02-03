#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "imu.h"
#include "motor.h"
#include <TaskScheduler.h>

extern double Kp, Ki, Kd;

// Global Variables
extern double currentAngle;
extern double targetAngle;
extern double output;
extern float safetyThreshold;

// TaskScheduler Declaration
Scheduler runner;
void controllerUpdate(); // Task function

// Task for PID control (10 ms interval)
int sampleTime = 10;
Task controlTask(sampleTime, TASK_FOREVER, &controllerUpdate);

void controllerInit() {
  imuInit();
  motorInit();

  // Initialize the scheduler and add the task
  runner.init();
  runner.addTask(controlTask);
  controlTask.enable();

  Serial.println("TaskScheduler started successfully.");
}

void controllerUpdate() {
  // Read current angle from IMU
  float rollAngle = getRoll();
  currentAngle = rollAngle; 
  // Safety check
  if (abs(rollAngle) > safetyThreshold) {
    stopMotor();
    Serial.println("Motor stopped due to excessive tilt.");
    targetAngle = 0.0;
    output = 0.0;
    return;
  }

  // Custom PID Calculation
  float error = rollAngle - targetAngle;
  static float errorSum = 0;
  static float lastError = 0;

  errorSum += error;
  errorSum = constrain(errorSum, -20, 20); // Anti-windup

  // Calculate derivative (no need for loopTime since TaskScheduler ensures fixed intervals)
  float derivative = (error - lastError) / (sampleTime / 1000.0); // sampleTime is in milliseconds
  lastError = error;
  output = Kp * error + Ki * errorSum + Kd * derivative;
  output = constrain(output, -MAX_RPM, MAX_RPM);
  setMotorVelocity(output); 

  // Debugging output
  Serial.print("Angle: ");
  Serial.print(rollAngle, 2);
  Serial.print(" deg | Output: ");
  Serial.print(output);
  Serial.print(" RPM | Motor Speed: ");
  Serial.print(getVelocityRPM());
  Serial.println(" RPM");
}

#endif
