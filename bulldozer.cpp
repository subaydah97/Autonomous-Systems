#include <Arduino.h> // Arduino framework
#include <Wire.h>    // I2C for the ToF sensor
#include <VL53L1X.h> // ToF distance sensor
#include <Servo.h>   // motor control via PWM "servo" pulses
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define _telemetry_target "bot/2"

const char *ssid = "mn-92937";
const char *password = "12345679";

const char *mqttServer = "192.168.212.106";
const char *OBSTACLE_TOPIC = "OR/NEW";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastTelemetryMs = 0;

const int TELEMETRY_INTERVAL_MS =
    1000 / 2; // Approximately 41 ms

bool firstTelemetryPrinted = false;
bool telemetryEnabled = false;

float obstacleX = 0.0f;
float obstacleY = 0.0f;
float latestZcm = 0.0f;
float signedPositionTicks = 0.0f;
float lastAvgTicksSeen = 0.0f;
bool positionInitialized = false;

// Fixed simulation start position
constexpr float START_X_CM = 0.0f;
constexpr float START_Y_CM = 0.0f;

// Obstacle target received from bot 1
float targetXcm = 0.0f;
float targetYcm = 0.0f;
float targetZcm = 0.0f;

bool newObstacleTargetReceived = false;

void connectWiFi()
{
    Serial.print("Connecting to WiFi");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void connectMQTT()
{
    mqttClient.setServer(mqttServer, 1883);

    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");

        // Must be different from bot 1
        if (mqttClient.connect("PicoW_Bulldozer"))
        {
            Serial.println("connected");

            mqttClient.subscribe(OBSTACLE_TOPIC);

            Serial.println("Subscribed to OR/NEW");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying...");

            delay(2000);
        }
    }
}

// =====================================================================
// ToF, motors and wheel-encoder correction
// =====================================================================

// ToF pins
#define TOF_SDA_PIN 4
#define TOF_SCL_PIN 5

// Motor pins
constexpr uint8_t LEFT_MOTOR_PIN = 6;
constexpr uint8_t RIGHT_MOTOR_PIN = 7;

// IR encoder output pins
constexpr uint8_t LEFT_ENCODER_PIN = 11;
constexpr uint8_t RIGHT_ENCODER_PIN = 14;

// Obstacle handling
constexpr uint16_t STOP_DISTANCE_MM = 50;

// Continuous-rotation servo calibration
constexpr int LEFT_STOP_US = 1500;
constexpr int RIGHT_STOP_US = 1500;

// These signs must make both wheels move forward for positive commands.
constexpr int LEFT_FORWARD_DIRECTION = 1;
constexpr int RIGHT_FORWARD_DIRECTION = -1;

constexpr int MAX_SERVO_OFFSET_US = 250;

// Calibrated base commands from the encoder test
constexpr float LEFT_BASE_SPEED = 0.418f;
constexpr float RIGHT_BASE_SPEED = 0.500f;

// Live straight-line correction
constexpr float ENCODER_KP = 0.25f;
constexpr float MAX_LIVE_CORRECTION = 0.06f;
constexpr float LEFT_CORRECTION_SCALE = 0.20f;
constexpr float RIGHT_CORRECTION_SCALE = 1.5f;

constexpr float MIN_DRIVE_SPEED = 0.30f;
constexpr float MAX_DRIVE_SPEED = 0.65f;

// The sensors produce only a few ticks per second, so one second
// provides a more useful comparison than a very short interval.
constexpr uint32_t ENCODER_CONTROL_INTERVAL_MS = 500;
constexpr uint32_t ENCODER_TIMEOUT_MS = 2500;
constexpr uint32_t MIN_ENCODER_INTERVAL_US = 3000;

// =====================================================================
// Hardware objects
// =====================================================================

VL53L1X sensor;
Servo leftMotor;
Servo rightMotor;

// =====================================================================
// Robot and movement state
// =====================================================================

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

RobotState robotState = WAITING_FOR_TARGET;
MotionMode motionMode = MotionMode::STOPPED;

// =====================================================================
// Encoder state
// =====================================================================

volatile uint32_t leftEncoderTicks = 0;
volatile uint32_t rightEncoderTicks = 0;

volatile uint32_t lastLeftEncoderEdgeUs = 0;
volatile uint32_t lastRightEncoderEdgeUs = 0;

uint32_t previousLeftTicks = 0;
uint32_t previousRightTicks = 0;

uint32_t lastEncoderControlMs = 0;
uint32_t lastLeftMovementMs = 0;
uint32_t lastRightMovementMs = 0;

// Encoder position when reversing starts
uint32_t reverseStartLeftTicks = 0;
uint32_t reverseStartRightTicks = 0;

// Number of forward ticks travelled
uint32_t forwardLeftTicks = 0;
uint32_t forwardRightTicks = 0;
bool pushingObstacle = false;
uint32_t pushStartTime = 0;

// =====================================================================
// Encoder interrupts
// =====================================================================

void leftEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    if ((nowUs - lastLeftEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        leftEncoderTicks++;
        lastLeftEncoderEdgeUs = nowUs;
    }
}

void rightEncoderInterrupt()
{
    const uint32_t nowUs = micros();

    if ((nowUs - lastRightEncoderEdgeUs) >= MIN_ENCODER_INTERVAL_US)
    {
        rightEncoderTicks++;
        lastRightEncoderEdgeUs = nowUs;
    }
}

void readEncoderTicks(uint32_t &leftTicks, uint32_t &rightTicks)
{
    noInterrupts();

    leftTicks = leftEncoderTicks;
    rightTicks = rightEncoderTicks;

    interrupts();
}

constexpr float WHEEL_TICK_TO_RAD = (2.0f * PI) / 10.0f;
constexpr float TICK_TO_CM = 0.72f * PI; // 1 tick ≈ 2.2619 cm (0.442 tick = 1 cm)

void getWheelRadians(
    float &leftRadians,
    float &rightRadians)
{
    uint32_t leftTicks;
    uint32_t rightTicks;

    readEncoderTicks(leftTicks, rightTicks);

    leftRadians = leftTicks * WHEEL_TICK_TO_RAD;
    rightRadians = rightTicks * WHEEL_TICK_TO_RAD;
} // <-- this closing brace MUST be here

void updateSignedPosition()
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    float avgTicks = (l + r) / 2.0f;

    if (!positionInitialized)
    {
        lastAvgTicksSeen = avgTicks;
        positionInitialized = true;
        return;
    }

    float deltaTicks = avgTicks - lastAvgTicksSeen;
    lastAvgTicksSeen = avgTicks;

    if (motionMode == MotionMode::BACKWARD)
    {
        signedPositionTicks -= deltaTicks;
    }
    else if (motionMode == MotionMode::FORWARD)
    {
        signedPositionTicks += deltaTicks;
    }
}
void resetPosition()
{
    uint32_t l, r;
    readEncoderTicks(l, r);

    signedPositionTicks = 0;
    lastAvgTicksSeen = (l + r) / 2.0f;
    positionInitialized = true;

    Serial.println("Position reset to X=0 Y=0");
}

void getLivePosition(float &liveX, float &liveY)
{
    liveX = START_X_CM + (signedPositionTicks * TICK_TO_CM * 0.1);
    liveY = START_Y_CM;
}

// =====================================================================
// Motor output
// =====================================================================

void writeMotorCommands(
    float leftSpeed,
    float rightSpeed,
    int movementDirection)
{
    leftSpeed = constrain(leftSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

    const int leftPulseUs =
        LEFT_STOP_US +
        static_cast<int>(
            LEFT_FORWARD_DIRECTION *
            movementDirection *
            leftSpeed *
            MAX_SERVO_OFFSET_US);

    const int rightPulseUs =
        RIGHT_STOP_US +
        static_cast<int>(
            RIGHT_FORWARD_DIRECTION *
            movementDirection *
            rightSpeed *
            MAX_SERVO_OFFSET_US);

    leftMotor.writeMicroseconds(leftPulseUs);
    rightMotor.writeMicroseconds(rightPulseUs);
}

void stopMotors()
{
    leftMotor.writeMicroseconds(LEFT_STOP_US);
    rightMotor.writeMicroseconds(RIGHT_STOP_US);
}

// =====================================================================
// Motion mode
// =====================================================================

void resetEncoderController()
{
    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;

    const uint32_t nowMs = millis();

    lastEncoderControlMs = nowMs;
    lastLeftMovementMs = nowMs;
    lastRightMovementMs = nowMs;
}

void setMotionMode(MotionMode newMode)
{

    if (motionMode == newMode)
        return;

    motionMode = newMode;
    telemetryEnabled =
        motionMode != MotionMode::STOPPED;

    if (motionMode == MotionMode::STOPPED)
    {
        stopMotors();
        Serial.println("Motion mode: STOPPED");
        return;
    }

    if (motionMode == MotionMode::FORWARD)
    {
        writeMotorCommands(
            LEFT_BASE_SPEED,
            RIGHT_BASE_SPEED,
            1);

        Serial.println("Motion mode: FORWARD");
    }
    else if (motionMode == MotionMode::BACKWARD)
    {
        // use your normal speed system but adjust imbalance here
        writeMotorCommands(
            LEFT_BASE_SPEED * 1.3f,
            RIGHT_BASE_SPEED * 0.30f, // RIGHT WHEEL SLOWER ADJUST THIS
            -1);

        Serial.println("Motion mode: BACKWARD (balanced)");
    }
}

// =====================================================================
// Encoder feedback controller
// =====================================================================

void updateWheelCorrection()
{
    if (motionMode == MotionMode::BACKWARD)
    {
        return; // disable all correction while reversing
    }
    if (motionMode == MotionMode::STOPPED)
    {
        return;
    }

    const uint32_t nowMs = millis();

    if (
        (nowMs - lastEncoderControlMs) < ENCODER_CONTROL_INTERVAL_MS)
    {
        return;
    }

    lastEncoderControlMs = nowMs;

    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    const int32_t leftTickChange =
        static_cast<int32_t>(
            currentLeftTicks - previousLeftTicks);

    const int32_t rightTickChange =
        static_cast<int32_t>(
            currentRightTicks - previousRightTicks);
    Serial.print("mode=");
    Serial.print((motionMode == MotionMode::FORWARD) ? "FWD" : (motionMode == MotionMode::BACKWARD) ? "BWD"
                                                                                                    : "STOP");

    Serial.print(" L=");
    Serial.print(leftTickChange);

    Serial.print(" R=");
    Serial.println(rightTickChange);

    previousLeftTicks = currentLeftTicks;
    previousRightTicks = currentRightTicks;

    if (leftTickChange > 0)
    {
        lastLeftMovementMs = nowMs;
    }

    if (rightTickChange > 0)
    {
        lastRightMovementMs = nowMs;
    }

    const bool leftEncoderTimedOut =
        (nowMs - lastLeftMovementMs) >
        ENCODER_TIMEOUT_MS;

    const bool rightEncoderTimedOut =
        (nowMs - lastRightMovementMs) >
        ENCODER_TIMEOUT_MS;

    if (leftEncoderTimedOut || rightEncoderTimedOut)
    {
        Serial.println();
        Serial.println("ENCODER FAULT");

        if (leftEncoderTimedOut)
        {
            Serial.println(
                "No movement detected from the left wheel.");
        }

        if (rightEncoderTimedOut)
        {
            Serial.println(
                "No movement detected from the right wheel.");
        }

        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    const float totalMovement =
        static_cast<float>(
            leftTickChange + rightTickChange);

    // Keep the previous commands when too few pulses were measured.
    if (totalMovement < 2.0f)
    {
        Serial.print("Encoder L=");
        Serial.print(currentLeftTicks);

        Serial.print(" R=");
        Serial.print(currentRightTicks);

        Serial.println(" | waiting for more pulses");
        return;
    }

    // Positive error means the left wheel was faster.
    const float normalizedError =
        static_cast<float>(
            leftTickChange - rightTickChange) /
        totalMovement;

    float liveCorrection =
        ENCODER_KP * normalizedError;

    liveCorrection = constrain(
        liveCorrection,
        -MAX_LIVE_CORRECTION,
        MAX_LIVE_CORRECTION);

    const float leftCommand =
        LEFT_BASE_SPEED +
        (liveCorrection * LEFT_CORRECTION_SCALE);

    const float rightCommand =
        RIGHT_BASE_SPEED -
        (liveCorrection * RIGHT_CORRECTION_SCALE);

    const int movementDirection =
        motionMode == MotionMode::FORWARD
            ? 1
            : -1;

    writeMotorCommands(
        leftCommand,
        rightCommand,
        movementDirection);
}

// =====================================================================
// Hardware setup
// =====================================================================

void setupMotors()
{
    leftMotor.attach(
        LEFT_MOTOR_PIN,
        1000,
        2000);

    rightMotor.attach(
        RIGHT_MOTOR_PIN,
        1000,
        2000);

    stopMotors();
    delay(1000);
}

void setupEncoders()
{
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    // FALLING is passed directly because the Pico Mbed framework
    // expects a PinStatus value, not an int constant.
    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_PIN),
        leftEncoderInterrupt,
        FALLING);

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        rightEncoderInterrupt,
        FALLING);

    Serial.println("Wheel encoders initialized.");
}

void setupToF()
{
    Wire.begin();
    Wire.setClock(100000);

    sensor.setTimeout(500);

    if (!sensor.init())
    {
        Serial.println("VL53L1X not found");

        while (true)
        {
            stopMotors();
            delay(1000);
        }
    }

    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(100);
}

// =====================================================================
// ToF and robot-state handling
// =====================================================================

void handleObstacleAvoidance()
{
    if (!sensor.dataReady())
    {
        return;
    }

    const uint16_t distance = sensor.read(false);

    if (sensor.timeoutOccurred())
    {
        Serial.println("ToF timeout");
        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }

    if (robotState == GOING_OUT)
    {
        float liveX, liveY;
        getLivePosition(liveX, liveY);

        if (!pushingObstacle && liveX >= targetXcm)
        {
            readEncoderTicks(forwardLeftTicks, forwardRightTicks);

            pushingObstacle = true;
            pushStartTime = millis();

            Serial.println("Reached obstacle.");

            Serial.print("Forward ticks L=");
            Serial.print(forwardLeftTicks);
            Serial.print(" R=");
            Serial.println(forwardRightTicks);
        }

        if (pushingObstacle &&
            millis() - pushStartTime >= 5000)
        {
            readEncoderTicks(
                reverseStartLeftTicks,
                reverseStartRightTicks);

            resetEncoderController();

            robotState = GOING_HOME;

            Serial.println("Finished pushing. Going home.");

            setMotionMode(MotionMode::BACKWARD);
        }
    }
    else if (robotState == GOING_HOME)
    {
        uint32_t leftTicks;
        uint32_t rightTicks;

        readEncoderTicks(leftTicks, rightTicks);

        int32_t leftBack =
            (int32_t)leftTicks -
            (int32_t)reverseStartLeftTicks;

        int32_t rightBack =
            (int32_t)rightTicks -
            (int32_t)reverseStartRightTicks;

        const uint32_t avgBack =
            (leftBack + rightBack) / 2;

        const uint32_t avgForward =
            (forwardLeftTicks + forwardRightTicks) / 2;

        if (avgBack >= avgForward && avgForward > 0)
        {
            robotState = HOME;
            pushingObstacle = false;

            Serial.println("Returned to start.");

            setMotionMode(MotionMode::STOPPED);
            signedPositionTicks = 0;
            lastAvgTicksSeen = 0;
        }
    }
}

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
    if (strcmp(topic, OBSTACLE_TOPIC) != 0)
    {
        return;
    }

    JsonDocument doc;

    DeserializationError error =
        deserializeJson(doc, payload, length);

    if (error)
    {
        Serial.print("Invalid obstacle JSON: ");
        Serial.println(error.c_str());
        return;
    }

    JsonObjectConst position =
        doc["position"].as<JsonObjectConst>();

    if (position.isNull())
    {
        Serial.println(
            "Obstacle message has no position.");
        return;
    }

    // Bot 1 sends meters.
    // Convert received values back to centimeters internally.
    targetXcm = position["x"].as<float>() * 100.0f;
    targetYcm = position["y"].as<float>() * 100.0f;
    targetZcm = position["z"].as<float>() * 100.0f;

    newObstacleTargetReceived = true;

    Serial.println("Obstacle target received.");

    Serial.print("Target in centimeters: x=");
    Serial.print(targetXcm);

    Serial.print(" y=");
    Serial.print(targetYcm);

    Serial.print(" z=");
    Serial.println(targetZcm);

    if (robotState == WAITING_FOR_TARGET)
    {
        newObstacleTargetReceived = false;
        pushingObstacle = false;

        robotState = GOING_OUT;

        resetEncoderController();

        Serial.println(
            "Starting bulldozer movement.");

        setMotionMode(MotionMode::FORWARD);
        resetPosition();
    }
}

// =====================================================================
// Main setup and loop
// =====================================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println(
        "Starting bulldozer.");

    setupMotors();
    setupEncoders();
    setupToF();

    mqttClient.setCallback(
        mqttCallback);

    connectWiFi();
    connectMQTT();

    robotState =
        WAITING_FOR_TARGET;
        resetPosition();

    Serial.println(
        "Bulldozer waiting for obstacle data...");
}

void loop()
{
    if (!mqttClient.connected())
    {
        connectMQTT();
    }

    mqttClient.loop();

    // =====================================================
    // BOT 2 TELEMETRY STREAM
    // =====================================================
    if (
        telemetryEnabled &&
        millis() - lastTelemetryMs >=
            TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = millis();

        JsonDocument botDoc;

        JsonObject pos =
            botDoc["position"]
                .to<JsonObject>();

        float liveX;
        float liveY;

        getLivePosition(liveX, liveY);

        pos["x"] = liveX * 0.01f;
        pos["y"] = liveY * 0.01f;
        pos["z"] = latestZcm * 0.01f;

        JsonObject wheels =
            botDoc["wheels"]
                .to<JsonObject>();

        float lRad;
        float rRad;

        getWheelRadians(
            lRad,
            rRad);

        wheels["radian_position_wheel_left"] = lRad;

        wheels["radian_position_wheel_right"] = rRad;

        char botBuffer[128];

        serializeJson(
            botDoc,
            botBuffer);

        mqttClient.publish(
            _telemetry_target,
            botBuffer);

        // Print only the first telemetry message
        if (!firstTelemetryPrinted)
        {
            Serial.println(
                "First bulldozer telemetry:");

            Serial.println(botBuffer);

            firstTelemetryPrinted = true;
        }
    }

    handleObstacleAvoidance();
    updateWheelCorrection();
    updateSignedPosition();
}
