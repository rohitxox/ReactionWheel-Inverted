#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <CAN.h>
#include <tinymovr.hpp>

//Motor configuration
#define CAN_SPEED 1000E3 // 1 Mbps
//#define MOTOR_ID 1
#define MOTOR_ID 2
#define PROTOCOL_HASH 0x700
#define MOTOR_VOLTAGE 24  
#define MOTOR_KV 300
int MAX_RPM = 1500;  // 7200 RPM

const float ticksPerRevolution = 8192.0;
// const float gearRatio = 10.0;
// const float maxRPM = 320.0;


//CAN callbacks
void sendCAN(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr) {
    CAN.beginExtendedPacket(arbitration_id, data_size, rtr);
    for (int i = 0; i < data_size; i++) {
        CAN.write(data[i]);
    }
    CAN.endPacket();
}

bool recvCAN(uint32_t *arbitration_id, uint8_t *data, uint8_t *data_size) {
    int packetSize = CAN.parsePacket();
    if (packetSize > 0) {
        *data_size = packetSize;
        for (int i = 0; i < packetSize; i++) {
            int r = CAN.read();
            if (r == -1) return false;
            data[i] = (uint8_t)r;
        }
        *arbitration_id = CAN.packetId();
        return true;
    }
    return false;
}

void delayMicrosecondsCustom(uint32_t us) {
    delayMicroseconds(us);
}

//Tinymovr instance
Tinymovr tinymovr(MOTOR_ID, sendCAN, recvCAN, delayMicrosecondsCustom, 100);

//Motor initialization
void motorInit() {
    if (!CAN.begin(CAN_SPEED)) {
        Serial.println("Starting CAN failed! Check connections.");
    }
    if (!CAN.filterExtended(0x0, PROTOCOL_HASH)) {
        Serial.println("Setting CAN filters failed!");
    }
    if (tinymovr.get_protocol_hash() != avlos_proto_hash) {
        Serial.println("Wrong device spec! Check firmware.");
    }
    //Enable closed-loop control
    tinymovr.controller.set_state(2);
}

// void motor(float rpm, bool reverse = false) {
//     // Convert RPM to ticks per second
//     float velocityTicks = (rpm * ticksPerRevolution * gearRatio) / 60.0;
//     if (reverse) velocityTicks = -velocityTicks;

//     // Set motor velocity in ticks per second
//     tinymovr.controller.velocity_mode();
//     tinymovr.controller.velocity.set_setpoint(velocityTicks);
// }

// Set motor velocity in RPM
void setMotorVelocity(float rpm, bool reverse = false) {
  if (reverse) rpm = -rpm; // Handle reverse direction
  rpm = constrain(rpm, -MAX_RPM, MAX_RPM); // Constrain RPM to motor limits

  // Convert RPM to ticks/s
  float velocityTicks = (rpm * ticksPerRevolution) / 60.0;

  // Set motor velocity in ticks/s
  tinymovr.controller.velocity_mode();
  tinymovr.controller.velocity.set_setpoint(velocityTicks);
}

//Stop motor
void stopMotor() {
    tinymovr.controller.velocity.set_setpoint(0);
}

float getVelocityRPM() {
    float actualVelTicks = tinymovr.sensors.user_frame.get_velocity_estimate(); 
    float motorRPM = (actualVelTicks * 60.0) / ticksPerRevolution; // No gear ratio
    return motorRPM;
}

#endif
