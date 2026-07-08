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
            uint32_t currentLeft;
            uint32_t currentRight;

            readEncoderTicks(currentLeft, currentRight);

            forwardLeftTicks =
                currentLeft - forwardStartLeftTicks;

            forwardRightTicks =
                currentRight - forwardStartRightTicks;

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

        Serial.print("Forward=");
        Serial.print(avgForward);

        Serial.print(" Back=");
        Serial.print(avgBack);

        Serial.print(" Lback=");
        Serial.print(leftBack);

        Serial.print(" Rback=");
        Serial.println(rightBack);

        if (avgBack >= avgForward && avgForward > 0)
        {
            pushingObstacle = false;

            Serial.println("Returned to start.");

            resetPosition();

            if (getNextTask())
            {
                Serial.println("Starting next queued task.");

                robotState = GOING_OUT;

                resetEncoderController();

                setMotionMode(MotionMode::FORWARD);
            }
            else
            {
                robotState = WAITING_FOR_TARGET;

                setMotionMode(MotionMode::STOPPED);

                Serial.println("Waiting for new tasks.");
            }
        }
    }
}

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
    if (strcmp(topic, OBSTACLE_TOPIC) != 0)
    {
        return;
    }

    JsonDocument doc;

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
    float x = position["x"].as<float>() * 100.0f;
    float y = position["y"].as<float>() * 100.0f;
    float z = position["z"].as<float>() * 100.0f;

    addTask(x, y, z);

    newObstacleTargetReceived = true;

    Serial.println("Obstacle target received.");

    Serial.print("Target in centimeters: x=");
    Serial.print(x);

    Serial.print(" y=");
    Serial.print(y);

    Serial.print(" z=");
    Serial.println(z);

    if (robotState == WAITING_FOR_TARGET)
    {
        if (getNextTask())
        {
            pushingObstacle = false;

            robotState = GOING_OUT;

            resetEncoderController();
            resetPosition();

            readEncoderTicks(
                forwardStartLeftTicks,
                forwardStartRightTicks);

            Serial.println("Starting next task.");

            setMotionMode(MotionMode::FORWARD);
        }
    }
}
