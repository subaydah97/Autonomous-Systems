/*
 * DriveSystem.h
 *
 * Controls the Bulldozer Chariot's drive system and onboard sensors.
 *
 * This header declares the motor, encoder, and Time-of-Flight (ToF)
 * sensor interfaces used for robot movement. It also provides
 * functions for motor control, wheel encoder feedback, position
 * tracking, motion mode management, and hardware initialization.
 */
#pragma once

#include <Arduino.h>
#include <VL53L1X.h>
#include <Servo.h>
#include "RobotState.h"

extern VL53L1X sensor;
extern Servo leftMotor;
extern Servo rightMotor;

// Encoder interrupt routines.
void leftEncoderInterrupt();
void rightEncoderInterrupt();

// Reads the current encoder tick counts.
void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks);

// Returns the wheel positions in radians.
void getWheelRadians(float &leftRadians, float &rightRadians);

// Updates the estimated robot position from the encoder ticks.
void updateSignedPosition();

// Resets the estimated robot position to the origin.
void resetPosition();

// Returns the current estimated robot position.
void getLivePosition(float &liveX, float &liveY);

// Sends speed commands to the motors.
void writeMotorCommands(float leftSpeed, float rightSpeed, int movementDirection);

// Stops both motors.
void stopMotors();

// Resets the encoder controller before a new movement.
void resetEncoderController();

// Changes the current motion mode.
void setMotionMode(MotionMode newMode);

// Uses encoder feedback to keep the robot driving straight.
void updateWheelCorrection();

// Initializes the motors.
void setupMotors();

// Initializes the wheel encoders.
void setupEncoders();

// Initializes the ToF distance sensor.
void setupToF();

// Adds a new obstacle location to the task queue.
void addTask(float x, float y, float z);

// Retrieves the next obstacle task from the queue.
bool getNextTask();

// Immediately stops the robot and blocks further movement.
void activateEmergencyStop();
