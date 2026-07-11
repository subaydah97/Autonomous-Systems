#include <Arduino.h>
#include <Wire.h>
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

// Time-of-Flight distance sensor used for obstacle detection.
VL53L1X sensor;

// Continuous rotation servo objects for the left and right drive wheels.
Servo leftMotor;
Servo rightMotor;

// Interrupt routine for the left wheel encoder.
// Every falling edge corresponds to one encoder pulse.
// A minimum time between pulses is enforced to filter out signal noise.
void leftEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    // Ignore pulses that arrive too quickly (debouncing/noise filtering).
    if ((nowUs - lastLeftEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        leftEncoderTicks++;
        lastLeftEncoderEdgeUs = nowUs;
    }
}

// Interrupt routine for the right wheel encoder.
// Counts valid encoder pulses while filtering noise.
void rightEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    if ((nowUs - lastRightEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        rightEncoderTicks++;
        lastRightEncoderEdgeUs = nowUs;
    }
}

// Safely reads the current encoder counts.
// Interrupts are temporarily disabled so both values are read consistently.
void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks)
{
    noInterrupts();

    leftTicks = leftEncoderTicks;
    rightTicks = rightEncoderTicks;

    interrupts();
}

// Converts normalized speed commands into servo pulse widths.
// movementDirection:
//   1  = forward
//  -1  = backward
void writeMotorCommands(
    float leftSpeed,
    float rightSpeed,
    int movementDirection)
{
    // Keep speeds within safe operating limits.
    leftSpeed = constrain(leftSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

    // Calculate servo pulse for the left motor.
    const int leftPulseUs =
        LEFT_STOP_US +
        static_cast<int>(
            LEFT_FORWARD_DIRECTION *
            movementDirection *
            leftSpeed *
            MAX_SERVO_OFFSET_US);

    // Calculate servo pulse for the right motor.
    const int rightPulseUs =
        RIGHT_STOP_US +
        static_cast<int>(
            RIGHT_FORWARD_DIRECTION *
            movementDirection *
            rightSpeed *
            MAX_SERVO_OFFSET_US);

    // Send commands to both motors.
    leftMotor.writeMicroseconds(leftPulseUs);
    rightMotor.writeMicroseconds(rightPulseUs);
}

// Stops both motors by sending their calibrated neutral pulse width.
void stopMotors()
{
    leftMotor.writeMicroseconds(LEFT_STOP_US);
    rightMotor.writeMicroseconds(RIGHT_STOP_US);
}

// Immediately stops the robot and blocks all future movement commands.
// This function is triggered when an emergency stop command is received.
void activateEmergencyStop()
{
    emergencyStopActive = true;
    telemetryEnabled = false;
    motionMode = MotionMode::STOPPED;

    stopMotors();

    Serial.println();
    Serial.println("EMERGENCY STOP ACTIVATED");
    Serial.println("Motor movement is now blocked.");
}

// Resets the encoder controller.
// The current encoder values become the new reference point for future
// wheel-speed comparisons and timeout detection.
void resetEncoderController()
{
    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;

    const uint32_t nowMs = millis();

    lastEncoderControlMs = nowMs;
    lastLeftMovementMs = nowMs;
    lastRightMovementMs = nowMs;
}

// Changes the robot's motion state and sends the corresponding
// motor commands.
void setMotionMode(MotionMode newMode)
{
    // Prevent any movement while the emergency stop is active.
    if (
        emergencyStopActive &&
        newMode != MotionMode::STOPPED)
    {
        Serial.println(
            "Movement blocked: emergency stop is active.");
        return;
    }

    // Ignore requests that do not change the current mode.
    if (motionMode == newMode)
        return;

    motionMode = newMode;

    // Telemetry is only transmitted while the robot is moving.
    telemetryEnabled = (motionMode != MotionMode::STOPPED);

    // Stop both motors.
    if (motionMode == MotionMode::STOPPED)
    {
        stopMotors();
        Serial.println("Motion mode: STOPPED");
        return;
    }

    // Drive forwards using the calibrated base speeds.
    if (motionMode == MotionMode::FORWARD)
    {
        writeMotorCommands(
            LEFT_BASE_SPEED,
            RIGHT_BASE_SPEED,
            1);

        Serial.println("Motion mode: FORWARD");
    }
    // Drive backwards.
    // Different speed multipliers compensate for mechanical differences
    // between the two wheels while reversing.
    else if (motionMode == MotionMode::BACKWARD)
    {
        writeMotorCommands(
            LEFT_BASE_SPEED * 1.3f,
            RIGHT_BASE_SPEED * 0.30f,
            -1);

        Serial.println("Motion mode: BACKWARD (balanced)");
    }
}

// Starts the return journey after the robot has reached its target.
// The current encoder values are stored as the starting point for
// measuring the distance travelled while reversing.
void startReverse()
{
    readEncoderTicks(reverseStartLeftTicks, reverseStartRightTicks);

    // Reset the encoder controller before changing direction.
    resetEncoderController();

    // Update the robot state to indicate that it is returning home.
    robotState = GOING_HOME;

    // Begin driving backwards.
    setMotionMode(MotionMode::BACKWARD);
}

// Converts the encoder tick counts into wheel rotation in radians.
// This information is used in the telemetry sent to the simulation.
void getWheelRadians(float &leftRad, float &rightRad)
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    leftRad = l * WHEEL_TICK_TO_RAD;
    rightRad = r * WHEEL_TICK_TO_RAD;
}

// Updates the estimated position of the robot using wheel encoder data.
// The average wheel movement is used as an approximation of the
// distance travelled along the x-axis.
void updateSignedPosition()
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    // Calculate the average encoder count of both wheels.
    float avgTicks = (l + r) / 2.0f;

    // Determine how many encoder ticks occurred since the previous update.
    float deltaTicks = avgTicks - lastAvgTicksSeen;
    lastAvgTicksSeen = avgTicks;

    // Moving forward increases the position,
    // while moving backward decreases it.
    if (motionMode == MotionMode::BACKWARD)
    {
        signedPositionTicks -= deltaTicks;
    }
    else if (motionMode == MotionMode::FORWARD)
    {
        signedPositionTicks += deltaTicks;
    }

    // No update is required while the robot is stopped.
}

// Returns the robot's estimated position in centimetres.
// The current implementation assumes movement only along the x-axis.
void getLivePosition(float &liveX, float &liveY)
{
    liveX = signedPositionTicks * TICK_TO_CM;
    liveY = 0.0f;
}

// Continuously compares the movement of both wheels and adjusts the
// motor speeds to keep the robot driving in a straight line.
void updateWheelCorrection()
{
    // Steering correction is disabled while reversing.
    if (motionMode == MotionMode::BACKWARD)
    {
        return;
    }

    // No correction is needed while the robot is stationary.
    if (motionMode == MotionMode::STOPPED)
    {
        return;
    }

    const uint32_t nowMs = millis();

    // Only perform a correction at fixed time intervals.
    if (
        (nowMs - lastEncoderControlMs) < ENCODER_CONTROL_INTERVAL_MS)
    {
        return;
    }

    lastEncoderControlMs = nowMs;

    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;

    // Read the latest encoder values.
    readEncoderTicks(currentLeftTicks, currentRightTicks);

    // Calculate how many encoder pulses each wheel produced
    // since the previous controller update.
    const int32_t leftTickChange =
        static_cast<int32_t>(
            currentLeftTicks - previousLeftTicks);

    const int32_t rightTickChange =
        static_cast<int32_t>(
            currentRightTicks - previousRightTicks);

    Serial.print("mode=");
    Serial.print((motionMode == MotionMode::FORWARD) ? "FWD" :
                 (motionMode == MotionMode::BACKWARD) ? "BWD" :
                 "STOP");

    Serial.print(" L=");
    Serial.print(leftTickChange);

    Serial.print(" R=");
    Serial.println(rightTickChange);

    // Store the current encoder values for the next comparison.
    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;

    // Record the last time movement was detected on each wheel.
    if (leftTickChange > 0)
    {
        lastLeftMovementMs = nowMs;
    }

    if (rightTickChange > 0)
    {
        lastRightMovementMs = nowMs;
    }

    // Detect encoder failures by checking if either wheel has
    // stopped producing pulses for too long.
    const bool leftEncoderTimedOut =
        (nowMs - lastLeftMovementMs) >
        ENCODER_TIMEOUT_MS;

    const bool rightEncoderTimedOut =
        (nowMs - lastRightMovementMs) >
        ENCODER_TIMEOUT_MS;

    if (leftEncoderTimedOut || rightEncoderTimedOut)
    {
        Serial.println();
        Serial.println("ENCODER FAULT");

        if (leftEncoderTimedOut)
        {
            Serial.println(
                "No movement detected from the left wheel.");
        }

        if (rightEncoderTimedOut)
        {
            Serial.println(
                "No movement detected from the right wheel.");
        }

        // Stop the robot if an encoder is no longer providing data.
        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    // Total number of encoder pulses detected during this update.
    const float totalMovement =
        static_cast<float>(
            leftTickChange + rightTickChange);

    // Ignore very small measurements because they are too noisy
    // to calculate a reliable correction.
    if (totalMovement < 2.0f)
    {
        Serial.print("Encoder L=");
        Serial.print(currentLeftTicks);

        Serial.print(" R=");
        Serial.print(currentRightTicks);

        Serial.println(" | waiting for more pulses");
        return;
    }

    // Calculate the difference between the two wheel speeds.
    // A positive value indicates that the left wheel is moving faster.
    const float normalizedError =
        static_cast<float>(
            leftTickChange - rightTickChange) /
        totalMovement;

    // Convert the wheel speed difference into a steering correction.
    float liveCorrection =
        ENCODER_KP * normalizedError;

    // Prevent excessively large corrections.
    liveCorrection = constrain(
        liveCorrection,
        -MAX_LIVE_CORRECTION,
        MAX_LIVE_CORRECTION);

    // Adjust the base motor speeds.
    const float leftCommand =
        LEFT_BASE_SPEED + (liveCorrection * LEFT_CORRECTION_SCALE);

    const float rightCommand =
        RIGHT_BASE_SPEED - (liveCorrection * RIGHT_CORRECTION_SCALE);

    const int movementDirection =
        motionMode == MotionMode::FORWARD
            ? 1
            : -1;

    // Send the corrected motor commands.
    writeMotorCommands(
        leftCommand,
        rightCommand,
        movementDirection);
}

// Initializes both continuous-rotation servo motors.
void setupMotors()
{
    // Attach the left motor to its GPIO pin.
    leftMotor.attach(
        LEFT_MOTOR_PIN,
        1000,
        2000);

    // Attach the right motor to its GPIO pin.
    rightMotor.attach(
        RIGHT_MOTOR_PIN,
        1000,
        2000);

    // Ensure the robot remains stationary during startup.
    stopMotors();

    // Allow the servos time to initialise before use.
    delay(1000);
}

// Configures the wheel encoder inputs and attaches the interrupt
// service routines used to count encoder pulses.
void setupEncoders()
{
    // Configure the encoder pins as inputs with internal pull-up resistors.
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    // Generate an interrupt on every falling edge of the encoder signal.
    // Each interrupt corresponds to one encoder pulse and increments
    // the wheel tick counter.
    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_PIN),
        leftEncoderInterrupt,
        FALLING);

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        rightEncoderInterrupt,
        FALLING);

    Serial.println("Wheel encoders initialized.");
}

// Initializes the VL53L1X Time-of-Flight distance sensor.
void setupToF()
{
    Wire.begin();
    Wire.setClock(100000);

    // Set the maximum time allowed for a sensor reading.
    sensor.setTimeout(500);

    // Stop the robot if the sensor cannot be detected.
    if (!sensor.init())
    {
        Serial.println("VL53L1X not found");

        while (true)
        {
            stopMotors();
            delay(1000);
        }
    }

    // Configure the sensor for long-range distance measurements.
    sensor.setDistanceMode(VL53L1X::Long);

    // Set the measurement timing budget (50 ms).
    sensor.setMeasurementTimingBudget(50000);

    // Begin continuous measurements every 100 ms.
    sensor.startContinuous(100);
}
