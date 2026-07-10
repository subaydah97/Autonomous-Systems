#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>
#include "ObstacleHandler.h"
#include "Communication.h"
#include "DriveSystem.h"
#include "RobotState.h"


void handleObstacleAvoidance()
{
    // Check if the ToF sensor has a new measurement available.
    // If no new measurement exists, nothing needs to be done.
    if (!sensor.dataReady())
    {
        return;
    }


    // Read the distance measurement from the VL53L1X sensor.
    // Currently this is mainly used for monitoring.
    const uint16_t distance = sensor.read(false);


    // Safety check:
    // If the ToF sensor fails, stop the robot.
    if (sensor.timeoutOccurred())
    {
        Serial.println("ToF timeout");

        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);

        return;
    }


    /*
     * GOING_OUT:
     *
     * The bulldozer is travelling toward the obstacle location
     * received from Bot 1.
     */
    if (robotState == GOING_OUT)
    {
        float liveX;
        float liveY;


        // Convert encoder ticks into the current position.
        // The bulldozer only moves on the X axis.
        getLivePosition(liveX, liveY);



        /*
         * Check if the bulldozer reached the target position.
         *
         * targetXcm comes from the MQTT obstacle message.
         *
         * Once reached:
         * - Save how far the bulldozer travelled.
         * - Start the pushing timer.
         */
        if (!pushingObstacle && liveX >= targetXcm)
        {
            uint32_t currentLeft;
            uint32_t currentRight;


            // Read current encoder values.
            readEncoderTicks(
                currentLeft,
                currentRight);



            // Calculate distance travelled from the starting point.
            //
            // The absolute encoder count is not used because the
            // robot may complete multiple missions.
            //
            // Only the movement during this mission matters.
            forwardLeftTicks =
                currentLeft - forwardStartLeftTicks;

            forwardRightTicks =
                currentRight - forwardStartRightTicks;


            // Enable pushing mode.
            pushingObstacle = true;


            // Start five second pushing timer.
            pushStartTime = millis();


            Serial.println("Reached obstacle.");

            Serial.print("Forward ticks L=");
            Serial.print(forwardLeftTicks);

            Serial.print(" R=");
            Serial.println(forwardRightTicks);
        }



        /*
         * Push phase:
         *
         * The bulldozer remains against the obstacle for 5 seconds.
         * After pushing:
         * - Store current encoder values as the reverse starting point.
         * - Change state to GOING_HOME.
         * - Drive backwards.
         */
        if (
            pushingObstacle &&
            millis() - pushStartTime >= 5000)
        {
            readEncoderTicks(
                reverseStartLeftTicks,
                reverseStartRightTicks);


            resetEncoderController();


            robotState = GOING_HOME;


            Serial.println(
                "Finished pushing. Going home.");


            setMotionMode(
                MotionMode::BACKWARD);
        }
    }



    /*
     * GOING_HOME:
     *
     * The bulldozer reverses until it has travelled the same
     * encoder distance as the forward movement.
     *
     * This allows the robot to return to the original position
     * without requiring GPS or external localization.
     */
    else if (robotState == GOING_HOME)
    {
        uint32_t leftTicks;
        uint32_t rightTicks;


        readEncoderTicks(
            leftTicks,
            rightTicks);



        /*
         * Calculate reverse distance.
         *
         * reverseStartLeftTicks/rightTicks were saved when
         * the bulldozer started reversing.
         */
        int32_t leftBack =
            (int32_t)leftTicks -
            (int32_t)reverseStartLeftTicks;

        int32_t rightBack =
            (int32_t)rightTicks -
            (int32_t)reverseStartRightTicks;



        // Average both wheels to reduce error.
        const uint32_t avgBack =
            (leftBack + rightBack) / 2;


        // Distance travelled toward obstacle.
        const uint32_t avgForward =
            (forwardLeftTicks + forwardRightTicks) / 2;



        // Debug encoder information.
        Serial.print("Forward=");
        Serial.print(avgForward);

        Serial.print(" Back=");
        Serial.print(avgBack);

        Serial.print(" Lback=");
        Serial.print(leftBack);

        Serial.print(" Rback=");
        Serial.println(rightBack);



        /*
         * When the reverse distance matches the forward distance,
         * the bulldozer has returned to its starting point.
         */
        if (
            avgBack >= avgForward &&
            avgForward > 0)
        {
            pushingObstacle = false;


            Serial.println(
                "Returned to start.");

            resetPosition();



            /*
             * Check if another obstacle task exists.
             *
             * If a task exists:
             *      start driving again.
             *
             * Otherwise:
             *      wait for another MQTT obstacle message.
             */
            if (getNextTask())
            {
                Serial.println(
                    "Starting next queued task.");


                robotState = GOING_OUT;


                resetEncoderController();


                setMotionMode(
                    MotionMode::FORWARD);
            }
            else
            {
                robotState = WAITING_FOR_TARGET;


                setMotionMode(
                    MotionMode::STOPPED);


                Serial.println(
                    "Waiting for new tasks.");
            }
        }
    }
}



/*
 * MQTT callback function.
 *
 * Receives:
 *
 * 1. Emergency stop commands:
 *      bot/2/COMMANDS
 *
 * 2. Obstacle coordinates:
 *      OR/NEW
 *
 */
void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{

    /*
     * Emergency stop handling.
     *
     * This is checked first because safety commands
     * must always have priority.
     */
    if (strcmp(topic, COMMAND_TOPIC) == 0)
    {
        String command;

        command.reserve(length);


        // Convert MQTT byte payload into a String.
        for (unsigned int i = 0; i < length; i++)
        {
            command += static_cast<char>(
                payload[i]);
        }


        command.trim();


        Serial.print(
            "Command received: ");

        Serial.println(command);



        if (command == "ESTOP")
        {
            activateEmergencyStop();
        }


        return;
    }



    /*
     * Ignore new tasks after emergency stop.
     */
    if (emergencyStopActive)
    {
        return;
    }



    /*
     * Ignore MQTT messages that are not obstacle messages.
     */
    if (strcmp(topic, OBSTACLE_TOPIC) != 0)
    {
        return;
    }



    /*
     * Decode obstacle JSON message from Bot 1.
     *
     * Example:
     *
     * {
     *   "position":{
     *       "x":1.25,
     *       "y":0,
     *       "z":0
     *   }
     * }
     */
    JsonDocument doc;


    DeserializationError error =
        deserializeJson(
            doc,
            payload,
            length);



    if (error)
    {
        Serial.print(
            "Invalid obstacle JSON: ");

        Serial.println(
            error.c_str());

        return;
    }



    JsonObjectConst position =
        doc["position"]
        .as<JsonObjectConst>();


    if (position.isNull())
    {
        Serial.println(
            "Obstacle message has no position.");

        return;
    }



    /*
     * Bot 1 sends values in meters.
     *
     * Internally the bulldozer works in centimeters,
     * so convert here.
     */
    float x =
        position["x"].as<float>() *
        100.0f;

    float y =
        position["y"].as<float>() *
        100.0f;

    float z =
        position["z"].as<float>() *
        100.0f;



    // Store task in queue.
    addTask(
        x,
        y,
        z);


    newObstacleTargetReceived = true;



    Serial.println(
        "Obstacle target received.");

    Serial.print(
        "Target in centimeters: x=");

    Serial.print(x);

    Serial.print(
        " y=");

    Serial.print(y);

    Serial.print(
        " z=");

    Serial.println(z);



    /*
     * If the bulldozer is idle,
     * immediately start the new task.
     */
    if (robotState == WAITING_FOR_TARGET)
    {
        if (getNextTask())
        {
            pushingObstacle = false;


            robotState = GOING_OUT;


            resetEncoderController();

            resetPosition();



            // Save starting encoder values.
            // Used later to calculate forward distance.
            readEncoderTicks(
                forwardStartLeftTicks,
                forwardStartRightTicks);



            Serial.println(
                "Starting next task.");



            setMotionMode(
                MotionMode::FORWARD);
        }
    }
}
