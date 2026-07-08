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

void writeMotorCommands(float leftSpeed, float rightSpeed, int movementDirection);
void stopMotors();
void resetEncoderController();
void setMotionMode(MotionMode newMode);
void startReverse();
void getWheelRadians(float &leftRad, float &rightRad);
void updateSignedPosition();
void getLivePosition(float &liveX, float &liveY);
void updateWheelCorrection();

void setupMotors();
void setupEncoders();
void setupToF();
void addTask(float x, float y, float z);
bool getNextTask();
