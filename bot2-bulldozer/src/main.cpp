#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"
#include "ObstacleHandler.h"
#include "RobotState.h"
#include "Telemetry.h"

void setup()
{
    // Start serial communication for debugging
    Serial.begin(115200);
    delay(2000);

    Serial.println(
        "Starting bulldozer.");


    // Initialize hardware systems
    setupMotors();      // Setup motor outputs
    setupEncoders();    // Setup wheel encoder interrupts
    setupToF();         // Setup distance sensor


    // Setup MQTT message callback
    mqttClient.setCallback(
        mqttCallback);

    // Connect robot to network
    connectWiFi();
    connectMQTT();


    // Start in waiting state until obstacle data arrives
    robotState =
        WAITING_FOR_TARGET;

    // Reset position tracking to starting point
    resetPosition();

    Serial.println(
        "Bulldozer waiting for obstacle data...");
}


void loop()
{
    // Reconnect MQTT if connection is lost
    if (!mqttClient.connected())
    {
        connectMQTT();
    }

    // Process incoming MQTT messages
    mqttClient.loop();


    // Stop all movement during emergency stop
    if (emergencyStopActive)
    {
        stopMotors();
        return;
    }


    // Send position and wheel telemetry
    handleTelemetry();

    // Handle obstacle pushing and return movement
    handleObstacleAvoidance();

    // Adjust motor speeds using encoder feedback
    updateWheelCorrection();

    // Update live position from encoder ticks
    updateSignedPosition();
}
