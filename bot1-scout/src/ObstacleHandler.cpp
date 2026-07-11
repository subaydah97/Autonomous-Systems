#include <Arduino.h>
#include "ObstacleHandler.h"
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

// Handles obstacle detection and the robot's behaviour
// while driving towards and returning from an obstacle.
void handleObstacleAvoidance()
{
    // Only read the ToF sensor when a new measurement is available.
    if (!sensor.dataReady())
    {
        return;
    }

    // Read the current distance measured by the ToF sensor.
    const uint16_t distance =
        sensor.read(false);

    // Stop the robot if communication with the sensor fails.
    if (sensor.timeoutOccurred())
    {
        Serial.println("ToF timeout");

        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    // =====================================================
    // Driving towards the obstacle
    // =====================================================
    if (robotState == GOING_OUT)
    {
        // Stop once the obstacle is detected within the
        // configured stopping distance.
        if (distance < STOP_DISTANCE_MM)
        {
            // Store the encoder values at the point where
            // the obstacle was reached.
            readEncoderTicks(forwardLeftTicks, forwardRightTicks);

            robotState = GOING_HOME;

            Serial.println("Obstacle detected.");
            setMotionMode(MotionMode::STOPPED);

            // Estimate the obstacle position from the average
            // encoder distance travelled.
            float avgForwardTicks =
                (forwardLeftTicks + forwardRightTicks) / 2.0f;

            obstacleX = avgForwardTicks * TICK_TO_CM;
            obstacleY = 0.0f;

            // Notify the telemetry module that obstacle
            // coordinates should be transmitted.
            waitingForCoordinates = true;
            coordinatesSent = false;
            waitStartTime = millis();

            // Debug information.
            Serial.print("Forward ticks L=");
            Serial.print(forwardLeftTicks);
            Serial.print(" R=");
            Serial.println(forwardRightTicks);
        }
    }

    // =====================================================
    // Driving back to the starting position
    // =====================================================
    else if (robotState == GOING_HOME)
    {
        uint32_t leftTicks;
        uint32_t rightTicks;

        readEncoderTicks(leftTicks, rightTicks);

        // Calculate how far each wheel has travelled while reversing.
        int32_t leftBack =
            (int32_t)leftTicks -
            (int32_t)reverseStartLeftTicks;

        int32_t rightBack =
            (int32_t)rightTicks -
            (int32_t)reverseStartRightTicks;

        // Compare the average reverse distance with the
        // forward distance travelled.
        const uint32_t avgBack =
            (leftBack + rightBack) / 2;

        const uint32_t avgForward =
            (forwardLeftTicks + forwardRightTicks) / 2;

        // Once the reverse distance matches the outward
        // distance, the robot has returned to its start.
        if (avgBack >= avgForward && avgForward > 0)
        {
            robotState = HOME;

            Serial.println("Returned to start.");

            setMotionMode(MotionMode::STOPPED);
        }
    }
}
