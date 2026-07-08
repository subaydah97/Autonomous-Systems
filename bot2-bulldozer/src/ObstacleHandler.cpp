#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>
#include "ObstacleHandler.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotState.h"

void handleObstacleAvoidance()
{
    if (!sensor.dataReady())
    {
        return;
    }

    const uint16_t distance = sensor.read(false);

    if (sensor.timeoutOccurred())
    {
        Serial.println("ToF timeout");
        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    if (robotState == GOING_OUT)
    {
        float liveX, liveY;
        getLivePosition(liveX, liveY);

        if (!pushingObstacle && liveX >= targetXcm)
        {
            readEncoderTicks(forwardLeftTicks, forwardRightTicks);

            pushingObstacle = true;
            pushStartTime = millis();

            Serial.println("Reached obstacle.");

            Serial.print("Forward ticks L=");
            Serial.print(forwardLeftTicks);
            Serial.print(" R=");
            Serial.println(forwardRightTicks);
        }

        if (pushingObstacle &&
            millis() - pushStartTime >= 5000)
        {
            readEncoderTicks(
                reverseStartLeftTicks,
                reverseStartRightTicks);

            resetEncoderController();

            robotState = GOING_HOME;

            Serial.println("Finished pushing. Going home.");

            setMotionMode(MotionMode::BACKWARD);
        }
    }
    else if (robotState == GOING_HOME)
    {
        uint32_t leftTicks;
        uint32_t rightTicks;

        readEncoderTicks(leftTicks, rightTicks);

        int32_t leftBack =
            (int32_t)leftTicks -
            (int32_t)reverseStartLeftTicks;

        int32_t rightBack =
            (int32_t)rightTicks -
            (int32_t)reverseStartRightTicks;

        const uint32_t avgBack =
            (leftBack + rightBack) / 2;

        const uint32_t avgForward =
            (forwardLeftTicks + forwardRightTicks) / 2;

        if (avgBack >= avgForward && avgForward > 0)
        {
            robotState = HOME;
            pushingObstacle = false;

            Serial.println("Returned to start.");

            setMotionMode(MotionMode::STOPPED);
            signedPositionTicks = 0;
            lastAvgTicksSeen = 0;
        }
    }
}

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
    // Emergency command for bot 2
    if (strcmp(topic, COMMAND_TOPIC) == 0)
    {
        String command;
        command.reserve(length);

        for (unsigned int i = 0; i < length; i++)
        {
            command += static_cast<char>(payload[i]);
        }

        command.trim();

        Serial.print("Command received: ");
        Serial.println(command);

        if (command == "ESTOP")
        {
            activateEmergencyStop();
        }

        return;
    }

    // Ignore obstacle messages after an emergency stop
    if (emergencyStopActive)
    {
        return;
    }

    // Existing obstacle-message check
    if (strcmp(topic, OBSTACLE_TOPIC) != 0)
    {
        return;
    }

    JsonDocument doc;

    // Keep the rest of the existing callback unchanged

    DeserializationError error =
        deserializeJson(doc, payload, length);

    if (error)
    {
        Serial.print("Invalid obstacle JSON: ");
        Serial.println(error.c_str());
        return;
    }

    JsonObjectConst position =
        doc["position"].as<JsonObjectConst>();

    if (position.isNull())
    {
        Serial.println(
            "Obstacle message has no position.");
        return;
    }

    // Bot 1 sends meters.
    // Convert received values back to centimeters internally.
    targetXcm = position["x"].as<float>() * 100.0f;
    targetYcm = position["y"].as<float>() * 100.0f;
    targetZcm = position["z"].as<float>() * 100.0f;

    newObstacleTargetReceived = true;

    Serial.println("Obstacle target received.");

    Serial.print("Target in centimeters: x=");
    Serial.print(targetXcm);

    Serial.print(" y=");
    Serial.print(targetYcm);

    Serial.print(" z=");
    Serial.println(targetZcm);

    if (robotState == WAITING_FOR_TARGET)
    {
        newObstacleTargetReceived = false;
        pushingObstacle = false;

        robotState = GOING_OUT;

        resetEncoderController();

        Serial.println(
            "Starting bulldozer movement.");

        setMotionMode(MotionMode::FORWARD);
        resetPosition();
    }
}
