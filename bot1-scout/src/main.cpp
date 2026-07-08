#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"
#include "ObstacleHandler.h"
#include "Telemetry.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);
    connectWiFi();
    connectMQTT();

    Serial.println(
        "Starting ToF and encoder correction");

    setupMotors();
    setupEncoders();
    setupToF();
    setMotionMode(MotionMode::FORWARD);
    resetEncoderController();

    Serial.println("Pico is driving.");
}

void loop()
{
    if (!mqttClient.connected())
    {
        connectMQTT();
    }

    mqttClient.loop();

    handleObstaclePublish();
    handleTelemetry();
    handleObstacleAvoidance();
    updateWheelCorrection();
    updateSignedPosition();
}
// git test