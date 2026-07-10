#include <Arduino.h>
#include <Wire.h>
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

// Time-of-Flight distance sensor used for obstacle detection.
VL53L1X sensor;

// Continuous rotation servo motors.
Servo leftMotor;
Servo rightMotor;

// Interrupt triggered by the LEFT wheel encoder.
void leftEncoderInterrupt()
{
    // Record current time in microseconds.
    // This is used to reject false pulses/noise.
    const uint32_t nowUs = micros();


    // Ignore pulses that happen too quickly after the previous one.
    // This prevents electrical noise from being counted as movement.
    if ((nowUs - lastLeftEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        // A valid encoder pulse was detected.
        leftEncoderTicks++;

        // Store the time of this pulse for the next noise check.
        lastLeftEncoderEdgeUs = nowUs;
    }
}


// Interrupt triggered by the RIGHT wheel encoder.
void rightEncoderInterrupt()
{
    const uint32_t nowUs = micros();


    // Same noise filtering as the left encoder.
    if ((nowUs - lastRightEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        // Count one valid wheel rotation step.
        rightEncoderTicks++;

        // Save timestamp of the encoder edge.
        lastRightEncoderEdgeUs = nowUs;
    }
}

// Encoder counters are modified inside interrupts.
// To safely copy these values, interrupts are temporarily disabled.
//
// Without this protection, the CPU could read a value while the
// interrupt is changing it, causing corrupted data.
//

void readEncoderTicks(
    uint32_t &leftTicks,
    uint32_t &rightTicks)
{
    // Prevent encoder interrupts while copying values.
    noInterrupts();


    // Copy the current encoder values.
    leftTicks = leftEncoderTicks;
    rightTicks = rightEncoderTicks;


    // Re-enable interrupts.
    interrupts();
}

// The bulldozer uses continuous rotation servos.
// Servo speed is controlled by sending a pulse width:
//
// 1500us  = stop
// >1500us = one direction
// <1500us = opposite direction
//
// The speed values are converted into servo pulse offsets.
// Calibration values in RobotConfig.h compensate for motor
// differences.
//

void writeMotorCommands(
    float leftSpeed,
    float rightSpeed,
    int movementDirection)
{

    // Keep commands inside safe operating limits.
    leftSpeed = constrain(
        leftSpeed,
        MIN_DRIVE_SPEED,
        MAX_DRIVE_SPEED);

    rightSpeed = constrain(
        rightSpeed,
        MIN_DRIVE_SPEED,
        MAX_DRIVE_SPEED);



    // Calculate left servo pulse.
    //
    // movementDirection:
    //     1  = forward
    //    -1  = backward
    //
    // LEFT_FORWARD_DIRECTION compensates for the physical
    // mounting direction of the motor.
    const int leftPulseUs =
        LEFT_STOP_US +
        static_cast<int>(
            LEFT_FORWARD_DIRECTION *
            movementDirection *
            leftSpeed *
            MAX_SERVO_OFFSET_US);



    // Calculate right servo pulse.
    // The sign is usually opposite because the motor is mounted
    // mirrored compared to the left motor.
    const int rightPulseUs =
        RIGHT_STOP_US +
        static_cast<int>(
            RIGHT_FORWARD_DIRECTION *
            movementDirection *
            rightSpeed *
            MAX_SERVO_OFFSET_US);



    // Send the calculated speed commands to the motors.
    leftMotor.writeMicroseconds(leftPulseUs);
    rightMotor.writeMicroseconds(rightPulseUs);
}



// Immediately stop both motors.
void stopMotors()
{
    leftMotor.writeMicroseconds(LEFT_STOP_US);
    rightMotor.writeMicroseconds(RIGHT_STOP_US);
}

// Called when a safety stop command is received.
void activateEmergencyStop()
{
    emergencyStopActive = true;

    telemetryEnabled = false;

    motionMode = MotionMode::STOPPED;


    // Stop motors immediately.
    stopMotors();


    Serial.println();
    Serial.println("EMERGENCY STOP ACTIVATED");
    Serial.println("Motor movement is now blocked.");
}

// Used when starting a new movement.
// The current encoder position becomes the new reference point.
// Future encoder differences are calculated relative to this.
// This prevents old movement data from affecting a new task.
void resetEncoderController()
{
    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;


    // Read current encoder positions.
    readEncoderTicks(
        currentLeftTicks,
        currentRightTicks);



    // Store current position as starting point.
    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;



    const uint32_t nowMs = millis();


    // Reset encoder timing checks.
    lastEncoderControlMs = nowMs;
    lastLeftMovementMs = nowMs;
    lastRightMovementMs = nowMs;
}

// Controls the operating direction of the bulldozer:
// STOPPED  -> motors disabled
// FORWARD  -> driving towards obstacle
// BACKWARD -> returning after pushing obstacle
void setMotionMode(MotionMode newMode)
{

    // Safety check:
    // Movement commands are ignored while emergency stop is active.
    if (
        emergencyStopActive &&
        newMode != MotionMode::STOPPED)
    {
        Serial.println(
            "Movement blocked: emergency stop is active.");

        return;
    }



    // Ignore duplicate commands.
    if (motionMode == newMode)
        return;



    // Update current movement state.
    motionMode = newMode;



    // Telemetry is only sent while the robot is moving.
    telemetryEnabled =
        (motionMode != MotionMode::STOPPED);



    // -------------------------------
    // STOP MODE
    // -------------------------------
    if (motionMode == MotionMode::STOPPED)
    {
        stopMotors();

        Serial.println(
            "Motion mode: STOPPED");

        return;
    }



    // -------------------------------
    // FORWARD MODE
    // -------------------------------
    if (motionMode == MotionMode::FORWARD)
    {
        writeMotorCommands(
            LEFT_BASE_SPEED,
            RIGHT_BASE_SPEED,
            1);


        Serial.println(
            "Motion mode: FORWARD");
    }



    // -------------------------------
    // BACKWARD MODE
    // -------------------------------
    else if (motionMode == MotionMode::BACKWARD)
    {

        // Reverse speed is manually balanced.
        // One wheel is compensated because both motors
        // do not have identical behaviour in reverse.
        writeMotorCommands(
            LEFT_BASE_SPEED * 1.3f,
            RIGHT_BASE_SPEED * 0.30f,
            -1);


        Serial.println(
            "Motion mode: BACKWARD (balanced)");
    }
}

// Called after the bulldozer has finished pushing an obstacle.
//
// The encoder position at the moment of reversing is saved.
// The bulldozer then drives backwards until the same amount of
// distance has been travelled in reverse.
//
// This allows the robot to return to its original starting point
// without needing GPS or external positioning.
//

void startReverse()
{
    // Save current encoder values as the reverse starting point.
    readEncoderTicks(
        reverseStartLeftTicks,
        reverseStartRightTicks);


    // Reset encoder controller timing.
    resetEncoderController();


    // Change robot state so obstacle handler knows the robot
    // is returning home.
    robotState = GOING_HOME;


    // Start driving backwards.
    setMotionMode(
        MotionMode::BACKWARD);
}

// Converts encoder ticks into wheel rotation in radians.
//
// Formula:
//
// radians = encoder ticks × radians per tick
//
// This value is published through telemetry.
//

void getWheelRadians(
    float &leftRad,
    float &rightRad)
{
    uint32_t l;
    uint32_t r;


    // Read current encoder counters.
    readEncoderTicks(
        l,
        r);


    // Convert ticks into wheel angle.
    leftRad =
        l * WHEEL_TICK_TO_RAD;

    rightRad =
        r * WHEEL_TICK_TO_RAD;
}

// Tracks the bulldozer's position along the X-axis.
//
// The robot only moves in one dimension:
//
// Forward  -> position increases
// Backward -> position decreases
//
// Encoder ticks are averaged between both wheels to reduce
// errors caused by small differences between motors.
//

void updateSignedPosition()
{
    uint32_t l;
    uint32_t r;


    // Get current wheel encoder values.
    readEncoderTicks(
        l,
        r);



    // Calculate average wheel movement.
    float avgTicks =
        (l + r) / 2.0f;



    // First update after startup:
    // store the initial encoder position.
    if (!positionInitialized)
    {
        lastAvgTicksSeen = avgTicks;

        positionInitialized = true;

        return;
    }



    // Calculate movement since previous update.
    float deltaTicks =
        avgTicks - lastAvgTicksSeen;



    // Store current position for next calculation.
    lastAvgTicksSeen = avgTicks;



    // Update signed position depending on direction.
    if (motionMode == MotionMode::BACKWARD)
    {
        signedPositionTicks -= deltaTicks;
    }
    else if (motionMode == MotionMode::FORWARD)
    {
        signedPositionTicks += deltaTicks;
    }
}

// Converts encoder position into real-world coordinates.
//
// The bulldozer only moves along the X-axis, therefore:
//
// X = travelled distance
// Y = always zero
//

void getLivePosition(
    float &liveX,
    float &liveY)
{
    // Convert encoder ticks into centimetres.
    liveX =
        signedPositionTicks *
        TICK_TO_CM *
        0.1;


    // Robot does not move sideways.
    liveY = 0.0f;
}

// Used when the bulldozer returns home.
//
// The current location becomes the new zero reference.
//

void resetPosition()
{
    uint32_t l;
    uint32_t r;


    readEncoderTicks(
        l,
        r);



    // Reset travelled distance.
    signedPositionTicks = 0;



    // Store current encoder position as reference.
    lastAvgTicksSeen =
        (l + r) / 2.0f;


    positionInitialized = true;


    Serial.println(
        "Position reset to X=0 Y=0");
}

// The bulldozer can receive multiple obstacle locations.
//
// Tasks are stored in a circular queue:
//
// queueTail -> where new tasks are added
// queueHead -> next task to execute
//
// This prevents new obstacle messages from being lost while
// the bulldozer is still handling another obstacle.
//

void addTask(
    float x,
    float y,
    float z)
{

    // Calculate next available queue position.
    int nextTail =
        (queueTail + 1) %
        MAX_TASKS;



    // Check if queue is full.
    if (nextTail == queueHead)
    {
        Serial.println(
            "Task queue full!");

        return;
    }



    // Store new obstacle coordinates.
    taskQueue[queueTail] =
    {
        x,
        y,
        z
    };


    // Move queue pointer forward.
    queueTail = nextTail;


    Serial.println(
        "Task added.");
}



bool getNextTask()
{

    // No tasks available.
    if (queueHead == queueTail)
    {
        return false;
    }



    // Load next obstacle coordinates.
    targetXcm =
        taskQueue[queueHead].x;

    targetYcm =
        taskQueue[queueHead].y;

    targetZcm =
        taskQueue[queueHead].z;



    // Move queue pointer forward.
    queueHead =
        (queueHead + 1) %
        MAX_TASKS;


    return true;
}


// Both motors are not identical.
// One motor may rotate slightly faster than the other.
//
// This function compares encoder movement:
//
// Left wheel ticks
// Right wheel ticks
//
// If one side moves faster, motor commands are adjusted to keep
// the bulldozer driving straight.
//
// This acts like a simple proportional controller.
//

void updateWheelCorrection()
{

    // Reverse movement does not use correction because reverse
    // has separate calibration.
    if (motionMode == MotionMode::BACKWARD)
    {
        return;
    }


    // Do nothing when stopped.
    if (motionMode == MotionMode::STOPPED)
    {
        return;
    }



    const uint32_t nowMs =
        millis();



    // Only update correction every set interval.
    if (
        (nowMs - lastEncoderControlMs)
        <
        ENCODER_CONTROL_INTERVAL_MS)
    {
        return;
    }



    lastEncoderControlMs = nowMs;



    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;



    // Read current encoder values.
    readEncoderTicks(
        currentLeftTicks,
        currentRightTicks);



    // Calculate movement since previous check.
    const int32_t leftTickChange =
        static_cast<int32_t>(
            currentLeftTicks -
            previousLeftTicks);


    const int32_t rightTickChange =
        static_cast<int32_t>(
            currentRightTicks -
            previousRightTicks);



    // Save current values for next cycle.
    previousLeftTicks =
        currentLeftTicks;

    previousRightTicks =
        currentRightTicks;



    // Update last movement times.
    if (leftTickChange > 0)
    {
        lastLeftMovementMs = nowMs;
    }

    if (rightTickChange > 0)
    {
        lastRightMovementMs = nowMs;
    }


    // If motors are commanded to move but an encoder stops
    // producing pulses, a wheel may be blocked or disconnected.
    //

    const bool leftEncoderTimedOut =
        (nowMs - lastLeftMovementMs)
        >
        ENCODER_TIMEOUT_MS;


    const bool rightEncoderTimedOut =
        (nowMs - lastRightMovementMs)
        >
        ENCODER_TIMEOUT_MS;



    if (
        leftEncoderTimedOut ||
        rightEncoderTimedOut)
    {

        Serial.println(
            "ENCODER FAULT");


        robotState =
            HOME;


        setMotionMode(
            MotionMode::STOPPED);


        return;
    }



    // Ignore very small movements because there are not enough
    // encoder samples to calculate reliable correction.
    const float totalMovement =
        static_cast<float>(
            leftTickChange +
            rightTickChange);



    if (totalMovement < 2.0f)
    {
        return;
    }



    // Positive value means left wheel moved faster.
    //
    // Example:
    // left = 10 ticks
    // right = 8 ticks
    //
    // Error = positive
    // Reduce left speed / increase right speed
    //

    const float normalizedError =
        static_cast<float>(
            leftTickChange -
            rightTickChange)
        /
        totalMovement;



    // Proportional correction.
    float liveCorrection =
        ENCODER_KP *
        normalizedError;



    // Limit correction strength.
    liveCorrection =
        constrain(
            liveCorrection,
            -MAX_LIVE_CORRECTION,
            MAX_LIVE_CORRECTION);



    // Apply correction to motor commands.
    const float leftCommand =
        LEFT_BASE_SPEED +
        (
            liveCorrection *
            LEFT_CORRECTION_SCALE
        );


    const float rightCommand =
        RIGHT_BASE_SPEED -
        (
            liveCorrection *
            RIGHT_CORRECTION_SCALE
        );



    const int movementDirection =
        motionMode == MotionMode::FORWARD
            ? 1
            : -1;



    // Send corrected motor speeds.
    writeMotorCommands(
        leftCommand,
        rightCommand,
        movementDirection);
}

void setupMotors()
{
    // Attach left motor servo signal.
    leftMotor.attach(
        LEFT_MOTOR_PIN,
        1000,
        2000);


    // Attach right motor servo signal.
    rightMotor.attach(
        RIGHT_MOTOR_PIN,
        1000,
        2000);



    // Safety measure:
    // Ensure motors are stopped before starting operation.
    stopMotors();


    // Allow servos to initialize.
    delay(1000);
}


// Initializes the wheel encoders.
//
// The encoders provide feedback about wheel movement by sending
// electrical pulses.
//
// Each pulse triggers an interrupt:
//     LEFT  -> leftEncoderInterrupt()
//     RIGHT -> rightEncoderInterrupt()
//
// These interrupts increase the encoder tick counters.
//
// The encoder data is later used for:
// - travelled distance calculation
// - wheel rotation telemetry
// - straight-line correction
// - detecting stalled wheels
//

void setupEncoders()
{

    // Encoder outputs are open collector signals.
    // Internal pull-up resistors keep the signal HIGH when idle.
    pinMode(
        LEFT_ENCODER_PIN,
        INPUT_PULLUP);


    pinMode(
        RIGHT_ENCODER_PIN,
        INPUT_PULLUP);



    // Attach interrupt for the left wheel encoder.
    //
    // FALLING means the interrupt triggers when the signal changes:
    // HIGH -> LOW
    //
    // Each falling edge represents one encoder tick.
    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_PIN),
        leftEncoderInterrupt,
        FALLING);



    // Attach interrupt for the right wheel encoder.
    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        rightEncoderInterrupt,
        FALLING);



    Serial.println(
        "Wheel encoders initialized.");
}

void setupToF()
{


    Wire.begin();
    Wire.setClock(100000);
    sensor.setTimeout(500);


    if (!sensor.init())
    {

        Serial.println(
            "VL53L1X not found");



        // If the sensor is missing, stop the robot permanently.
        //
        // Without distance sensing the bulldozer cannot safely
        // detect obstacles.
        while (true)
        {
            stopMotors();

            delay(1000);
        }
    }

    sensor.setDistanceMode(
        VL53L1X::Long);
        
    sensor.setMeasurementTimingBudget(
        50000);

    sensor.startContinuous(
        100);
}
