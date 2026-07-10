/*
 * RobotState.h
 *
 * Defines the shared state variables for Bot 2 (Bulldozer Chariot).
 *
 * This header contains the robot's operating states, motion modes,
 * and global variables used to track navigation, encoder feedback,
 * telemetry, obstacle positions, and motor control. These variables
 * are shared across multiple source files to coordinate the robot's
 * behaviour.
 */

#pragma once

#include <Arduino.h>

// Describes the current stage of the robot's task.
enum RobotState
{
    WAITING_FOR_TARGET, // Waiting for a new obstacle location.
    GOING_OUT,          // Driving towards the obstacle.
    GOING_HOME,         // Returning after pushing the obstacle.
    HOME                // Robot has returned to its start position.
};

// Describes the current motor movement direction.
enum class MotionMode
{
    STOPPED,
    FORWARD,
    BACKWARD
};

// Controls telemetry timing and output.
extern unsigned long lastTelemetryMs;
extern bool firstTelemetryPrinted;
extern bool telemetryEnabled;

// Stores the detected obstacle position.
extern float obstacleX;
extern float obstacleY;
extern float latestZcm;

// Encoder-based position estimation.
extern float signedPositionTicks;
extern float lastAvgTicksSeen;
extern bool positionInitialized;

// Current target received from the obstacle registry.
extern float targetXcm;
extern float targetYcm;
extern float targetZcm;
extern bool newObstacleTargetReceived;

// Prevents motor movement after an emergency stop.
extern bool emergencyStopActive;

extern RobotState robotState;
extern MotionMode motionMode;

// Updated by encoder interrupt routines.
extern volatile uint32_t leftEncoderTicks;
extern volatile uint32_t rightEncoderTicks;

// Used to prevent false encoder pulses caused by noise.
extern volatile uint32_t lastLeftEncoderEdgeUs;
extern volatile uint32_t lastRightEncoderEdgeUs;

// Previous encoder readings used for speed correction.
extern uint32_t previousLeftTicks;
extern uint32_t previousRightTicks;

// Encoder controller timing variables.
extern uint32_t lastEncoderControlMs;
extern uint32_t lastLeftMovementMs;
extern uint32_t lastRightMovementMs;

// Encoder values recorded when reversing.
extern uint32_t reverseStartLeftTicks;
extern uint32_t reverseStartRightTicks;

// Encoder values recorded while moving towards the obstacle.
extern uint32_t forwardLeftTicks;
extern uint32_t forwardRightTicks;

// Indicates whether the bulldozer is currently pushing an obstacle.
extern bool pushingObstacle;

// Stores when the pushing action started.
extern uint32_t pushStartTime;

// Structure containing an obstacle target location.
struct Task
{
    float x;
    float y;
    float z;
};

// Maximum number of queued obstacle tasks.
constexpr int MAX_TASKS = 10;

// Circular queue storing incoming obstacle locations.
extern Task taskQueue[MAX_TASKS];

// Queue indexes.
extern int queueHead;
extern int queueTail;

// Encoder values captured at the start of a forward movement.
extern uint32_t forwardStartLeftTicks;
extern uint32_t forwardStartRightTicks;
