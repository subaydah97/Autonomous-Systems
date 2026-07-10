/*
 * DriveSystem.h
 *
 * Controls the Scout Chariot's drive system and onboard sensors.
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

// Hardware devices
extern VL53L1X sensor;
extern Servo leftMotor;
extern Servo rightMotor;

// Wheel encoder interface
void leftEncoderInterrupt();
void rightEncoderInterrupt();
void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks);

// Motor control and motion management
void writeMotorCommands(float leftSpeed, float rightSpeed, int movementDirection);
void stopMotors();
void resetEncoderController();
void setMotionMode(MotionMode newMode);
void startReverse();

// Position and odometry
void getWheelRadians(float &leftRad, float &rightRad);
void updateSignedPosition();
void getLivePosition(float &liveX, float &liveY);
void updateWheelCorrection();

// Hardware initialization and safety
void setupMotors();
void setupEncoders();
void setupToF();
void activateEmergencyStop();
