#include <Arduino.h>
#include <ArduinoJson.h>
#include "Telemetry.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

void handleTelemetry()
{
    // BOT 2 TELEMETRY STREAM
    if (
        telemetryEnabled &&
        millis() - lastTelemetryMs >=
            TELEMETRY_INTERVAL_MS)
    {
        // Store time of last telemetry update
        lastTelemetryMs = millis();

        // Create JSON document for MQTT message
        JsonDocument botDoc;

        // Create position object inside JSON
        JsonObject pos =
            botDoc["position"]
                .to<JsonObject>();

        float liveX;
        float liveY;

        // Get current calculated robot position from encoders
        getLivePosition(liveX, liveY);

        // Convert cm internally to meters for MQTT
        pos["x"] = liveX * 0.01f;
        pos["y"] = liveY * 0.01f;
        pos["z"] = latestZcm * 0.01f;

        // Create wheel object inside JSON
        JsonObject wheels =
            botDoc["wheels"]
                .to<JsonObject>();

        float lRad;
        float rRad;

        // Convert encoder ticks into wheel rotation radians
        getWheelRadians(
            lRad,
            rRad);

        wheels["radian_position_wheel_left"] = lRad;
        wheels["radian_position_wheel_right"] = rRad;

        // Convert JSON document into character buffer
        char botBuffer[128];

        serializeJson(
            botDoc,
            botBuffer);

        // Publish telemetry message to bulldozer topic
        mqttClient.publish(
            _telemetry_target,
            botBuffer);

        // Only print the first telemetry message to avoid flooding Serial
        if (!firstTelemetryPrinted)
        {
            Serial.println(
                "First bulldozer telemetry:");

            Serial.println(botBuffer);

            firstTelemetryPrinted = true;
        }
    }
}
