#include <Arduino.h>
#include <Wire.h>
#include "DriveSystem.h"
#include "RobotConfig.h"
#include "RobotState.h"

VL53L1X sensor;
Servo leftMotor;
Servo rightMotor;

void leftEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    if ((nowUs - lastLeftEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        leftEncoderTicks++;
        lastLeftEncoderEdgeUs = nowUs;
    }
}

void rightEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    if ((nowUs - lastRightEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        rightEncoderTicks++;
        lastRightEncoderEdgeUs = nowUs;
    }
}

void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks)
{
    noInterrupts();

    leftTicks = leftEncoderTicks;
    rightTicks = rightEncoderTicks;

    interrupts();
}

void getWheelRadians(
    float &leftRadians,
    float &rightRadians)
{
    uint32_t leftTicks;
    uint32_t rightTicks;

    readEncoderTicks(leftTicks, rightTicks);

    leftRadians = leftTicks * WHEEL_TICK_TO_RAD;
    rightRadians = rightTicks * WHEEL_TICK_TO_RAD;
}

void updateSignedPosition()
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    float avgTicks = (l + r) / 2.0f;

    if (!positionInitialized)
    {
        lastAvgTicksSeen = avgTicks;
        positionInitialized = true;
        return;
    }

    float deltaTicks = avgTicks - lastAvgTicksSeen;
    lastAvgTicksSeen = avgTicks;

    if (motionMode == MotionMode::BACKWARD)
    {
        signedPositionTicks -= deltaTicks;
    }
    else if (motionMode == MotionMode::FORWARD)
    {
        signedPositionTicks += deltaTicks;
    }
}

void resetPosition()
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    signedPositionTicks = 0;
    lastAvgTicksSeen = (l + r) / 2.0f;
    positionInitialized = true;

    Serial.println("Position reset to X=0 Y=0");
}

void getLivePosition(float &liveX, float &liveY)
{
    liveX = START_X_CM + (signedPositionTicks * TICK_TO_CM * 0.1);
    liveY = START_Y_CM;
}

void writeMotorCommands(
    float leftSpeed,
    float rightSpeed,
    int movementDirection)
{
    leftSpeed = constrain(leftSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

    const int leftPulseUs =
        LEFT_STOP_US +
        static_cast<int>(
            LEFT_FORWARD_DIRECTION *
            movementDirection *
            leftSpeed *
            MAX_SERVO_OFFSET_US);

    const int rightPulseUs =
        RIGHT_STOP_US +
        static_cast<int>(
            RIGHT_FORWARD_DIRECTION *
            movementDirection *
            rightSpeed *
            MAX_SERVO_OFFSET_US);

    leftMotor.writeMicroseconds(leftPulseUs);
    rightMotor.writeMicroseconds(rightPulseUs);
}

void stopMotors()
{
    leftMotor.writeMicroseconds(LEFT_STOP_US);
    rightMotor.writeMicroseconds(RIGHT_STOP_US);
}

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

void setMotionMode(MotionMode newMode)
{
    if (
        emergencyStopActive &&
        newMode != MotionMode::STOPPED)
    {
        Serial.println(
            "Movement blocked: emergency stop is active.");
        return;
    }

    if (motionMode == newMode)
        return;

    motionMode = newMode;
    telemetryEnabled =
        motionMode != MotionMode::STOPPED;

    if (motionMode == MotionMode::STOPPED)
    {
        stopMotors();
        Serial.println("Motion mode: STOPPED");
        return;
    }

    if (motionMode == MotionMode::FORWARD)
    {
        writeMotorCommands(
            LEFT_BASE_SPEED,
            RIGHT_BASE_SPEED,
            1);

        Serial.println("Motion mode: FORWARD");
    }
    else if (motionMode == MotionMode::BACKWARD)
    {
        // use your normal speed system but adjust imbalance here
        writeMotorCommands(
            LEFT_BASE_SPEED * 1.3f,
            RIGHT_BASE_SPEED * 0.30f, // RIGHT WHEEL SLOWER ADJUST THIS
            -1);

        Serial.println("Motion mode: BACKWARD (balanced)");
    }
}

void updateWheelCorrection()
{
    if (motionMode == MotionMode::BACKWARD)
    {
        return; // disable all correction while reversing
    }
    if (motionMode == MotionMode::STOPPED)
    {
        return;
    }

    const uint32_t nowMs = millis();

    if (
        (nowMs - lastEncoderControlMs) < ENCODER_CONTROL_INTERVAL_MS)
    {
        return;
    }

    lastEncoderControlMs = nowMs;

    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    const int32_t leftTickChange =
        static_cast<int32_t>(
            currentLeftTicks - previousLeftTicks);

    const int32_t rightTickChange =
        static_cast<int32_t>(
            currentRightTicks - previousRightTicks);
    Serial.print("mode=");
    Serial.print((motionMode == MotionMode::FORWARD) ? "FWD" : (motionMode == MotionMode::BACKWARD) ? "BWD"
                                                                                                    : "STOP");

    Serial.print(" L=");
    Serial.print(leftTickChange);

    Serial.print(" R=");
    Serial.println(rightTickChange);

    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;

    if (leftTickChange > 0)
    {
        lastLeftMovementMs = nowMs;
    }

    if (rightTickChange > 0)
    {
        lastRightMovementMs = nowMs;
    }

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

        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    const float totalMovement =
        static_cast<float>(
            leftTickChange + rightTickChange);

    // Keep the previous commands when too few pulses were measured.
    if (totalMovement < 2.0f)
    {
        Serial.print("Encoder L=");
        Serial.print(currentLeftTicks);

        Serial.print(" R=");
        Serial.print(currentRightTicks);

        Serial.println(" | waiting for more pulses");
        return;
    }

    // Positive error means the left wheel was faster.
    const float normalizedError =
        static_cast<float>(
            leftTickChange - rightTickChange) /
        totalMovement;

    float liveCorrection =
        ENCODER_KP * normalizedError;

    liveCorrection = constrain(
        liveCorrection,
        -MAX_LIVE_CORRECTION,
        MAX_LIVE_CORRECTION);

    const float leftCommand =
        LEFT_BASE_SPEED +
        (liveCorrection * LEFT_CORRECTION_SCALE);

    const float rightCommand =
        RIGHT_BASE_SPEED -
        (liveCorrection * RIGHT_CORRECTION_SCALE);

    const int movementDirection =
        motionMode == MotionMode::FORWARD
            ? 1
            : -1;

    writeMotorCommands(
        leftCommand,
        rightCommand,
        movementDirection);
}

void setupMotors()
{
    leftMotor.attach(
        LEFT_MOTOR_PIN,
        1000,
        2000);

    rightMotor.attach(
        RIGHT_MOTOR_PIN,
        1000,
        2000);

    stopMotors();
    delay(1000);
}

void setupEncoders()
{
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    // FALLING is passed directly because the Pico Mbed framework
    // expects a PinStatus value, not an int constant.
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

void setupToF()
{
    Wire.begin();
    Wire.setClock(100000);

    sensor.setTimeout(500);

    if (!sensor.init())
    {
        Serial.println("VL53L1X not found");

        while (true)
        {
            stopMotors();
            delay(1000);
        }
    }

    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(100);
}
