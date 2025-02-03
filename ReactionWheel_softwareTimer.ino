#include "controller.h"
#include "matlab.h"

// PID Gains
double Kp = 12.0;
double Ki = 4.0;
double Kd = 1.5;

// Global Variables
double currentAngle = 0.0;
double targetAngle = 0.0;
double output = 0.0;
float safetyThreshold = 35.0;

void setup() {
  Serial.begin(115200);
  controllerInit();
  //matlabInit(); 
}

void loop() {
  //to handle periodic task
  runner.execute();
  //Prepare data to send
  // float angle = currentAngle;        
  // float pidOutput = output;    
  // float motorSpeed = getVelocityRPM();

  // sendDataToSimulink(angle, pidOutput, motorSpeed);
  // receiveDataFromSimulink();

}