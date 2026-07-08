#include "RobotState.h"

bool waitingForCoordinates = false;
bool coordinatesSent = false;
unsigned long waitStartTime = 0;
unsigned long lastTelemetryMs = 0;
bool firstTelemetryPrinted = false;
bool telemetryEnabled = false;

float latestZ = 0.0f;
float obstacleX = 0.0f;
float obstacleY = 0.0f; // stays 0, robot only moves along x
float signedPositionTicks = 0.0f;
float lastAvgTicksSeen = 0.0f;

RobotState robotState = GOING_OUT;
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

bool emergencyStopActive = false;