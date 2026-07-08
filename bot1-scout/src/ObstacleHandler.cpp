#include <Arduino.h>
#include "ObstacleHandler.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

void handleObstacleAvoidance()
{
    if (!sensor.dataReady())
    {
        return;
    }

    const uint16_t distance =
        sensor.read(false);

    if (sensor.timeoutOccurred())
    {
        Serial.println("ToF timeout");

        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    if (robotState == GOING_OUT)
    {
        if (distance < STOP_DISTANCE_MM)
        {
            readEncoderTicks(forwardLeftTicks, forwardRightTicks);

            robotState = GOING_HOME;

            Serial.println("Obstacle detected.");
            setMotionMode(MotionMode::STOPPED);

            float avgForwardTicks = (forwardLeftTicks + forwardRightTicks) / 2.0f;
            obstacleX = avgForwardTicks * TICK_TO_CM;
            obstacleY = 0.0f;

            waitingForCoordinates = true;
            coordinatesSent = false;
            waitStartTime = millis();

            Serial.print("Forward ticks L=");
            Serial.print(forwardLeftTicks);
            Serial.print(" R=");
            Serial.println(forwardRightTicks);
        }
    }
    else if (robotState == GOING_HOME)
    {
        uint32_t leftTicks;
        uint32_t rightTicks;

        readEncoderTicks(leftTicks, rightTicks);

        int32_t leftBack = (int32_t)leftTicks - (int32_t)reverseStartLeftTicks;
        int32_t rightBack = (int32_t)rightTicks - (int32_t)reverseStartRightTicks;

        const uint32_t avgBack = (leftBack + rightBack) / 2;
        const uint32_t avgForward = (forwardLeftTicks + forwardRightTicks) / 2;

        if (avgBack >= avgForward && avgForward > 0)
        {
            robotState = HOME;
            Serial.println("Returned to start.");
            setMotionMode(MotionMode::STOPPED);
        }
    }
}
