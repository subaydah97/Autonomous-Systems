#include <Arduino.h>
#include <ArduinoJson.h>
#include "Telemetry.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

void handleObstaclePublish()
{
    // =====================================================
    // 1. OBSTACLE PUBLISH (ONE TIME ONLY)
    // =====================================================
    if (waitingForCoordinates && !coordinatesSent)
    {
        if (millis() - waitStartTime >= 5000)
        {
            JsonDocument doc;

            JsonObject position = doc["position"].to<JsonObject>();
            position["x"] = obstacleX / 100.0f;
            position["y"] = obstacleY / 100.0f;
            position["z"] = latestZ / 100.0f;

            char buffer[128];
            serializeJson(doc, buffer);

            mqttClient.publish("OR/NEW", buffer);

            Serial.println("Obstacle sent:");
            Serial.println(buffer);

            coordinatesSent = true;
            waitingForCoordinates = false;

            startReverse();
        }
    }
}

void handleTelemetry()
{
    // =====================================================
    // 2. TELEMETRY STREAM (24 Hz ALWAYS)
    // =====================================================
    if (
        telemetryEnabled &&
        millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = millis();

        JsonDocument botDoc;

        float liveX, liveY;
        getLivePosition(liveX, liveY);

        JsonObject pos = botDoc["position"].to<JsonObject>();
        pos["x"] = liveX / 100.0f;
        pos["y"] = liveY / 100.0f;
        pos["z"] = latestZ / 100.0f;

        JsonObject wheels = botDoc["wheels"].to<JsonObject>();

        float lRad, rRad;
        getWheelRadians(lRad, rRad);

        wheels["radian_position_wheel_left"] = lRad;
        wheels["radian_position_wheel_right"] = rRad;

        char botBuffer[128];
        serializeJson(botDoc, botBuffer);

        mqttClient.publish("bot/1", botBuffer);

        // Print de eerste telemetrypositie maar één keer
        if (!firstTelemetryPrinted)
        {
            Serial.println("First telemetry position:");
            Serial.println(botBuffer);

            firstTelemetryPrinted = true;
        }
    }
}
