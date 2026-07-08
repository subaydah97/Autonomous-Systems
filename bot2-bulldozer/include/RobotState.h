#pragma once

#include <Arduino.h>

enum RobotState
{
    WAITING_FOR_TARGET,
    GOING_OUT,
    GOING_HOME,
    HOME
};

enum class MotionMode
{
    STOPPED,
    FORWARD,
    BACKWARD
};

extern unsigned long lastTelemetryMs;
extern bool firstTelemetryPrinted;
extern bool telemetryEnabled;

extern float obstacleX;
extern float obstacleY;
extern float latestZcm;
extern float signedPositionTicks;
extern float lastAvgTicksSeen;
extern bool positionInitialized;

extern float targetXcm;
extern float targetYcm;
extern float targetZcm;
extern bool newObstacleTargetReceived;

extern RobotState robotState;
extern MotionMode motionMode;

extern volatile uint32_t leftEncoderTicks;
extern volatile uint32_t rightEncoderTicks;
extern volatile uint32_t lastLeftEncoderEdgeUs;
extern volatile uint32_t lastRightEncoderEdgeUs;

extern uint32_t previousLeftTicks;
extern uint32_t previousRightTicks;
extern uint32_t lastEncoderControlMs;
extern uint32_t lastLeftMovementMs;
extern uint32_t lastRightMovementMs;
extern uint32_t reverseStartLeftTicks;
extern uint32_t reverseStartRightTicks;
extern uint32_t forwardLeftTicks;
extern uint32_t forwardRightTicks;
extern bool pushingObstacle;
extern uint32_t pushStartTime;

struct Task
{
    float x;
    float y;
    float z;
};

constexpr int MAX_TASKS = 10;

extern Task taskQueue[MAX_TASKS];
extern int queueHead;
extern int queueTail;

extern uint32_t forwardStartLeftTicks;
extern uint32_t forwardStartRightTicks;
