#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"
#include "ObstacleHandler.h"
#include "Telemetry.h"
#include "RobotState.h"

void setup()
{
    // Initialize the serial monitor for debugging.
    Serial.begin(115200);
    delay(2000);

    // Register the MQTT callback function for incoming messages.
    mqttClient.setCallback(mqttCallback);

    // Connect to the Wi-Fi network and MQTT broker.
    connectWiFi();
    connectMQTT();

    Serial.println(
        "Starting ToF and encoder correction");

    // Initialize the motors, wheel encoders and ToF distance sensor.
    setupMotors();
    setupEncoders();
    setupToF();

    // Start the robot moving forward and initialize the
    // encoder controller.
    setMotionMode(MotionMode::FORWARD);
    resetEncoderController();

    Serial.println("Pico is driving.");
}

void loop()
{
    // Reconnect to the MQTT broker if the connection is lost.
    if (!mqttClient.connected())
    {
        connectMQTT();
    }

    // Process incoming MQTT messages.
    mqttClient.loop();

    // Immediately stop all movement if an emergency stop
    // command has been received.
    if (emergencyStopActive)
    {
        stopMotors();
        return;
    }

    // Execute the robot's main control tasks.
    handleObstaclePublish();     // Publish detected obstacle coordinates.
    handleTelemetry();           // Send robot telemetry.
    handleObstacleAvoidance();   // Monitor for obstacles.
    updateWheelCorrection();     // Keep the robot driving straight.
    updateSignedPosition();      // Update the estimated robot position.
}
