#include "RobotState.h"

unsigned long lastTelemetryMs = 0;
bool firstTelemetryPrinted = false;
bool telemetryEnabled = false;

float obstacleX = 0.0f;
float obstacleY = 0.0f;
float latestZcm = 0.0f;
float signedPositionTicks = 0.0f;
float lastAvgTicksSeen = 0.0f;
bool positionInitialized = false;

Task taskQueue[MAX_TASKS];

int queueHead = 0;
int queueTail = 0;

// Current target
float targetXcm = 0.0f;
float targetYcm = 0.0f;
float targetZcm = 0.0f;

bool newObstacleTargetReceived = false;

RobotState robotState = WAITING_FOR_TARGET;
MotionMode motionMode = MotionMode::STOPPED;

volatile uint32_t leftEncoderTicks = 0;
volatile uint32_t rightEncoderTicks = 0;
volatile uint32_t lastLeftEncoderEdgeUs = 0;
volatile uint32_t lastRightEncoderEdgeUs = 0;

uint32_t previousLeftTicks = 0;
uint32_t previousRightTicks = 0;
uint32_t lastEncoderControlMs = 0;
uint32_t lastLeftMovementMs = 0;
uint32_t lastRightMovementMs = 0;
uint32_t reverseStartLeftTicks = 0;
uint32_t reverseStartRightTicks = 0;
uint32_t forwardLeftTicks = 0;
uint32_t forwardRightTicks = 0;
bool pushingObstacle = false;
uint32_t pushStartTime = 0;

bool emergencyStopActive = false;
uint32_t forwardStartLeftTicks = 0;
uint32_t forwardStartRightTicks = 0;
