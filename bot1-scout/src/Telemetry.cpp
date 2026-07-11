#include <Arduino.h>
#include <ArduinoJson.h>
#include "Telemetry.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

// Publishes the detected obstacle to the obstacle registry
// once the robot has stopped in front of it.
void handleObstaclePublish()
{
    // Publish only if an obstacle has been detected
    // and its coordinates have not yet been sent.
    if (waitingForCoordinates && !coordinatesSent)
    {
        // Wait a few seconds before publishing to allow
        // the robot to come to a complete stop.
        if (millis() - waitStartTime >= 5000)
        {
            JsonDocument doc;

            // Store the obstacle position in metres,
            // as expected by the MQTT registry.
            JsonObject position = doc["position"].to<JsonObject>();
            position["x"] = obstacleX / 100.0f;
            position["y"] = obstacleY / 100.0f;
            position["z"] = latestZ / 100.0f;

            char buffer[128];
            serializeJson(doc, buffer);

            // Publish the obstacle to the registry.
            mqttClient.publish("OR/NEW", buffer);

            Serial.println("Obstacle sent:");
            Serial.println(buffer);

            // Prevent the obstacle from being published again.
            coordinatesSent = true;
            waitingForCoordinates = false;

            // Begin the return journey.
            startReverse();
        }
    }
}

// Continuously publishes the robot's telemetry while it is moving.
void handleTelemetry()
{
    // Send telemetry only while enabled and at the configured interval.
    if (
        telemetryEnabled &&
        millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = millis();

        JsonDocument botDoc;

        // Obtain the current estimated robot position.
        float liveX, liveY;
        getLivePosition(liveX, liveY);

        // Store the robot position in metres.
        JsonObject pos = botDoc["position"].to<JsonObject>();
        pos["x"] = liveX / 100.0f;
        pos["y"] = liveY / 100.0f;
        pos["z"] = latestZ / 100.0f;

        // Include the wheel encoder positions in radians.
        JsonObject wheels = botDoc["wheels"].to<JsonObject>();

        float lRad, rRad;
        getWheelRadians(lRad, rRad);

        wheels["radian_position_wheel_left"] = lRad;
        wheels["radian_position_wheel_right"] = rRad;

        char botBuffer[128];
        serializeJson(botDoc, botBuffer);

        // Publish the telemetry message over MQTT.
        mqttClient.publish("bot/1", botBuffer);

        // Print only the first telemetry packet for debugging.
        if (!firstTelemetryPrinted)
        {
            Serial.println("First telemetry position:");
            Serial.println(botBuffer);

            firstTelemetryPrinted = true;
        }
    }
}
