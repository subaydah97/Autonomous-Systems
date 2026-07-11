#include "RobotState.h"

// Indicates whether obstacle coordinates still need to be published.
bool waitingForCoordinates = false;

// Prevents publishing the same obstacle more than once.
bool coordinatesSent = false;

// Timestamp used to delay actions after obstacle detection.
unsigned long waitStartTime = 0;

// Timestamp of the last telemetry transmission.
unsigned long lastTelemetryMs = 0;

// Used to print only the first telemetry message for debugging.
bool firstTelemetryPrinted = false;

// Enables or disables periodic telemetry transmission.
bool telemetryEnabled = false;

// Latest measured obstacle height (z-coordinate).
float latestZ = 0.0f;

// Estimated obstacle position in centimetres.
float obstacleX = 0.0f;
float obstacleY = 0.0f; // Robot only moves along the x-axis.

// Position estimated from encoder ticks.
float signedPositionTicks = 0.0f;

// Previous average encoder value used to calculate movement.
float lastAvgTicksSeen = 0.0f;

// Current navigation state.
RobotState robotState = GOING_OUT;

// Current motion mode of the drive system.
MotionMode motionMode = MotionMode::STOPPED;

// Updated by the encoder interrupt routines.
volatile uint32_t leftEncoderTicks = 0;
volatile uint32_t rightEncoderTicks = 0;

// Store the timestamp of the last valid encoder pulse
// to filter switch bounce and noise.
volatile uint32_t lastLeftEncoderEdgeUs = 0;
volatile uint32_t lastRightEncoderEdgeUs = 0;

// Previous encoder readings used for wheel speed correction.
uint32_t previousLeftTicks = 0;
uint32_t previousRightTicks = 0;

// Timing variables for the encoder controller.
uint32_t lastEncoderControlMs = 0;
uint32_t lastLeftMovementMs = 0;
uint32_t lastRightMovementMs = 0;

// Encoder values recorded when reverse motion starts.
uint32_t reverseStartLeftTicks = 0;
uint32_t reverseStartRightTicks = 0;

// Encoder values recorded when the obstacle is reached.
uint32_t forwardLeftTicks = 0;
uint32_t forwardRightTicks = 0;

// Becomes true after an emergency stop command is received.
bool emergencyStopActive = false;
