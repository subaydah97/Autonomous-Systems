#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"
#include "ObstacleHandler.h"
#include "RobotState.h"
#include "Telemetry.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println(
        "Starting bulldozer.");

    setupMotors();
    setupEncoders();
    setupToF();

    mqttClient.setCallback(
        mqttCallback);

    connectWiFi();
    connectMQTT();

    robotState =
        WAITING_FOR_TARGET;
    resetPosition();

    Serial.println(
        "Bulldozer waiting for obstacle data...");
}

void loop()
{
    if (!mqttClient.connected())
    {
        connectMQTT();
    }

    mqttClient.loop();

    if (emergencyStopActive)
    {
        stopMotors();
        return;
    }

    handleTelemetry();
    handleObstacleAvoidance();
    updateWheelCorrection();
    updateSignedPosition();
}
