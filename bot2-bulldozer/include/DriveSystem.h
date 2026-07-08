#pragma once

#include <Arduino.h>
#include <VL53L1X.h>
#include <Servo.h>
#include "RobotState.h"

extern VL53L1X sensor;
extern Servo leftMotor;
extern Servo rightMotor;

void leftEncoderInterrupt();
void rightEncoderInterrupt();
void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks);
void getWheelRadians(float &leftRadians, float &rightRadians);
void updateSignedPosition();
void resetPosition();
void getLivePosition(float &liveX, float &liveY);

void writeMotorCommands(float leftSpeed, float rightSpeed, int movementDirection);
void stopMotors();
void resetEncoderController();
void setMotionMode(MotionMode newMode);
void updateWheelCorrection();

void setupMotors();
void setupEncoders();
void setupToF();
