#include <Arduino.h>
#include <ArduinoJson.h>
#include "Telemetry.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

void handleTelemetry()
{
    // =====================================================
    // BOT 2 TELEMETRY STREAM
    // =====================================================
    if (
        telemetryEnabled &&
        millis() - lastTelemetryMs >=
            TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = millis();

        JsonDocument botDoc;

        JsonObject pos =
            botDoc["position"]
                .to<JsonObject>();

        float liveX;
        float liveY;

        getLivePosition(liveX, liveY);

        pos["x"] = liveX * 0.01f;
        pos["y"] = liveY * 0.01f;
        pos["z"] = latestZcm * 0.01f;

        JsonObject wheels =
            botDoc["wheels"]
                .to<JsonObject>();

        float lRad;
        float rRad;

        getWheelRadians(
            lRad,
            rRad);

        wheels["radian_position_wheel_left"] = lRad;

        wheels["radian_position_wheel_right"] = rRad;

        char botBuffer[128];

        serializeJson(
            botDoc,
            botBuffer);

        mqttClient.publish(
            _telemetry_target,
            botBuffer);

        // Print only the first telemetry message
        if (!firstTelemetryPrinted)
        {
            Serial.println(
                "First bulldozer telemetry:");

            Serial.println(botBuffer);

            firstTelemetryPrinted = true;
        }
    }
}
