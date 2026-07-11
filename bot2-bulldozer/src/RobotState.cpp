#include "RobotState.h"

// Telemetry control variables
unsigned long lastTelemetryMs = 0;
bool firstTelemetryPrinted = false;
bool telemetryEnabled = false;

// Received obstacle position
float obstacleX = 0.0f;
float obstacleY = 0.0f;
float latestZcm = 0.0f;

// Current robot position tracking
float signedPositionTicks = 0.0f;
float lastAvgTicksSeen = 0.0f;
bool positionInitialized = false;

// Emergency stop status
bool emergencyStopActive = false;


// Queue for storing multiple obstacle tasks
Task taskQueue[MAX_TASKS];

int queueHead = 0;
int queueTail = 0;


// Current target position
float targetXcm = 0.0f;
float targetYcm = 0.0f;
float targetZcm = 0.0f;

// New obstacle message received flag
bool newObstacleTargetReceived = false;


// Current robot state and movement mode
RobotState robotState = WAITING_FOR_TARGET;
MotionMode motionMode = MotionMode::STOPPED;


// Encoder counters (updated by interrupts)
volatile uint32_t leftEncoderTicks = 0;
volatile uint32_t rightEncoderTicks = 0;

// Time of last encoder pulse
volatile uint32_t lastLeftEncoderEdgeUs = 0;
volatile uint32_t lastRightEncoderEdgeUs = 0;


// Previous encoder values for correction calculations
uint32_t previousLeftTicks = 0;
uint32_t previousRightTicks = 0;

// Encoder timing control
uint32_t lastEncoderControlMs = 0;
uint32_t lastLeftMovementMs = 0;
uint32_t lastRightMovementMs = 0;


// Encoder values when reversing home
uint32_t reverseStartLeftTicks = 0;
uint32_t reverseStartRightTicks = 0;


// Distance travelled while pushing obstacle
uint32_t forwardLeftTicks = 0;
uint32_t forwardRightTicks = 0;


// Obstacle pushing control
bool pushingObstacle = false;
uint32_t pushStartTime = 0;


// Encoder values at mission start
uint32_t forwardStartLeftTicks = 0;
uint32_t forwardStartRightTicks = 0;
