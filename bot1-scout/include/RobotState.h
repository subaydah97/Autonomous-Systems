/*
 * RobotState.h
 *
 * Defines the shared state variables for Bot 1 (Scout Chariot).
 *
 * This header contains the robot's operating states, motion modes,
 * and global variables used to track navigation, encoder feedback,
 * telemetry, obstacle positions, and motor control. These variables
 * are shared across multiple source files to coordinate the robot's
 * behaviour.
 */

// Robot operating states
enum RobotState
{
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

// Navigation and telemetry state
extern bool waitingForCoordinates;
extern bool coordinatesSent;
extern unsigned long waitStartTime;
extern unsigned long lastTelemetryMs;
extern bool firstTelemetryPrinted;
extern bool telemetryEnabled;
extern bool emergencyStopActive;

// Position tracking
extern float latestZ;
extern float obstacleX;
extern float obstacleY;
extern float signedPositionTicks;
extern float lastAvgTicksSeen;

// Current robot status
extern RobotState robotState;
extern MotionMode motionMode;

// Encoder counters
extern volatile uint32_t leftEncoderTicks;
extern volatile uint32_t rightEncoderTicks;
extern volatile uint32_t lastLeftEncoderEdgeUs;
extern volatile uint32_t lastRightEncoderEdgeUs;

// Encoder controller variables
extern uint32_t previousLeftTicks;
extern uint32_t previousRightTicks;
extern uint32_t lastEncoderControlMs;
extern uint32_t lastLeftMovementMs;
extern uint32_t lastRightMovementMs;
extern uint32_t reverseStartLeftTicks;
extern uint32_t reverseStartRightTicks;
extern uint32_t forwardLeftTicks;
extern uint32_t forwardRightTicks;
