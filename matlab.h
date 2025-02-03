#ifndef MATLAB_H
#define MATLAB_H

#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "controller.h"

WiFiUDP udp;
const char* ssid = "Open";
const char* password = "fame@789";
const char* hostIP = "192.168.149.68"; // Simulink host IP
const int hostPort = 8080; // Simulink listener
const int localPort = 8081;

void matlabInit() {
    // Initialize WiFi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected!");

    // Initialize UDP
    udp.begin(localPort);
    Serial.print("Arduino IP Address: ");
    Serial.println(WiFi.localIP());
}

// Send data to Simulink
void sendDataToSimulink(float angle, float pidOutput, float motorSpeed) {
    float data[3] = {angle, pidOutput, motorSpeed};
    udp.beginPacket(hostIP, hostPort);
    udp.write((byte*)&data, sizeof(data));
    udp.endPacket();
    delay(10); // Match Simulink's sample time
}

// Receive data from Simulink
void receiveDataFromSimulink() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        float receivedData[3];  // Array to store Kp, Ki, Kd
        int bytesRead = udp.read((byte*)&receivedData, sizeof(receivedData));
        if (bytesRead == sizeof(receivedData)) {
            // Update PID gains
            Kp = receivedData[0];
            Ki = receivedData[1];
            Kd = receivedData[2];

            // Debugging
            Serial.print("Updated PID gains: ");
            Serial.print("Kp = ");
            Serial.print(Kp);
            Serial.print(", Ki = ");
            Serial.print(Ki);
            Serial.print(", Kd = ");
            Serial.println(Kd);
        } else {
            Serial.println("Invalid packet size received.");
        }
    }
}

#endif