#include <Arduino.h>    // Arduino framework
#include <BTstackLib.h> // BTstack Bluetooth stack (requires a Pico W board for the radio)
#include <SPI.h>        // required by BTstackLib
#include <Wire.h>       // I2C for the ToF sensor
#include <VL53L1X.h>    // ToF distance sensor
#include <Servo.h>      // motor control via PWM "servo" pulses
 
 
#define BEACON_PREFIX "ForestBeacon"                  // prefix used to filter beacon names in the scanner
#define BEACON_PREFIX_LEN (sizeof(BEACON_PREFIX) - 1) // length excluding the null terminator
 
// beacon coordinates in the environment, used for trilateration
const float BX[3] = {0.0f, 100.0f, 50.0f};
const float BY[3] = {0.0f, 0.0f, 100.0f};
 
// calibrated 1-unit baseline transmission powers (from tx_power_calibration.cpp)
const float TX_POWER[3] = {-15.32f, -28.98f, -18.87f};
const float ALPHA = 0.35f; // smoothing factor for the RSSI exponential moving average
const float n = 2.5f;      // path loss exponent for distance calculation
float rssi_avg[3] = {-100.0f, -100.0f, -100.0f};
bool seen[3] = {false, false, false};
 
struct Kalman2D
{
    float x, y, vx, vy; // state: position (x,y) and velocity (vx,vy)
    float P[4][4];      // covariance matrix
 
    float Q_pos; // position process noise
    float Q_vel; // velocity process noise
    float R;     // measurement noise
 
    bool initialised;
    unsigned long last_ms;
 
    void init(float start_x, float start_y)
    {
        x = start_x;
        y = start_y;
        vx = 0;
        vy = 0;
 
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                P[i][j] = (i == j) ? 500.0f : 0.0f;
 
        Q_pos = 0.1f;
        Q_vel = 0.3f;
        R = 30.0f;
 
        initialised = true;
        last_ms = millis();
    }
 
    void update(float mx, float my)
    {
        if (!initialised)
        {
            init(mx, my);
            return;
        }
 
        float dt = (millis() - last_ms) / 1000.0f;
        last_ms = millis();
        if (dt <= 0 || dt > 2.0f)
            dt = 0.1f;
 
        // state prediction
        float px = x + vx * dt;
        float py = y + vy * dt;
        float pvx = vx;
        float pvy = vy;
 
        // propagate covariance
        float PP[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                PP[i][j] = P[i][j];
 
        for (int j = 0; j < 4; j++)
        {
            PP[0][j] = P[0][j] + dt * P[2][j];
            PP[1][j] = P[1][j] + dt * P[3][j];
        }
        for (int i = 0; i < 4; i++)
        {
            P[i][0] = PP[i][0] + dt * PP[i][2];
            P[i][1] = PP[i][1] + dt * PP[i][3];
            P[i][2] = PP[i][2];
            P[i][3] = PP[i][3];
        }
        float dt2 = dt * dt;
        P[0][0] += Q_pos * dt2;
        P[1][1] += Q_pos * dt2;
        P[2][2] += Q_vel * dt2;
        P[3][3] += Q_vel * dt2;
 
        // innovation
        float innov_x = mx - px;
        float innov_y = my - py;
 
        float S00 = P[0][0] + R;
        float S11 = P[1][1] + R;
        float S01 = P[0][1];
 
        float det = S00 * S11 - S01 * S01;
        if (fabsf(det) < 1e-6f)
        {
            x = px;
            y = py;
            vx = pvx;
            vy = pvy;
            return;
        }
        float Si00 = S11 / det;
        float Si11 = S00 / det;
        float Si01 = -S01 / det;
 
        // Kalman gain
        float K[4][2];
        for (int i = 0; i < 4; i++)
        {
            K[i][0] = P[i][0] * Si00 + P[i][1] * Si01;
            K[i][1] = P[i][0] * Si01 + P[i][1] * Si11;
        }
 
        // updated state
        x = px + K[0][0] * innov_x + K[0][1] * innov_y;
        y = py + K[1][0] * innov_x + K[1][1] * innov_y;
        vx = pvx + K[2][0] * innov_x + K[2][1] * innov_y;
        vy = pvy + K[3][0] * innov_x + K[3][1] * innov_y;
 
        // updated covariance
        float newP[4][4];
        for (int i = 0; i < 4; i++)
        {
            newP[i][0] = P[i][0] - K[i][0] * P[0][0] - K[i][1] * P[1][0];
            newP[i][1] = P[i][1] - K[i][0] * P[0][1] - K[i][1] * P[1][1];
            newP[i][2] = P[i][2] - K[i][0] * P[0][2] - K[i][1] * P[1][2];
            newP[i][3] = P[i][3] - K[i][0] * P[0][3] - K[i][1] * P[1][3];
        }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                P[i][j] = newP[i][j];
    }
} kf;
 
static int beacon_index(const char *name)
{
    if (strcmp(name, "ForestBeaconOne") == 0)
        return 0;
    if (strcmp(name, "ForestBeaconTwo") == 0)
        return 1;
    if (strcmp(name, "ForestBeaconThree") == 0)
        return 2;
    return -1;
}
 
static float rssi_to_distance(int idx, float rssi)
{
    return powf(10.0f, (TX_POWER[idx] - rssi) / (10.0f * n));
}
 
static bool trilaterate(float *out_x, float *out_y)
{
    float d1 = rssi_to_distance(0, rssi_avg[0]);
    float d2 = rssi_to_distance(1, rssi_avg[1]);
    float d3 = rssi_to_distance(2, rssi_avg[2]);
 
    float x1 = BX[0], y1 = BY[0], x2 = BX[1], y2 = BY[1], x3 = BX[2], y3 = BY[2];
 
    float A = 2 * (x2 - x1);
    float B = 2 * (y2 - y1);
    float C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
    float D = 2 * (x3 - x1);
    float E = 2 * (y3 - y1);
    float F = d2 * d2 - d3 * d3 - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;
 
    float det = A * E - B * D;
    if (fabsf(det) < 0.001f)
        return false;
 
    *out_x = (C * E - F * B) / det;
    *out_y = (A * F - D * C) / det;
    return true;
}
 
static bool parse_name(const uint8_t *data, uint8_t max_len, char *out, uint8_t out_size)
{
    int i = 0;
    while (i < max_len)
    {
        uint8_t chunk_len = data[i];
        if (chunk_len == 0 || i + chunk_len > max_len)
            break;
        uint8_t chunk_type = data[i + 1];
        if (chunk_type == 0x09 || chunk_type == 0x08)
        {
            uint8_t name_len = chunk_len - 1;
            if (name_len >= out_size)
                name_len = out_size - 1;
            memcpy(out, &data[i + 2], name_len);
            out[name_len] = '\0';
            return true;
        }
        i += 1 + chunk_len;
    }
    return false;
}
 
void advertisementCallback(BLEAdvertisement *adv)
{
    char name[32] = {0};
    if (!parse_name(adv->getAdvData(), 31, name, sizeof(name)))
        return;
    if (strncmp(name, BEACON_PREFIX, BEACON_PREFIX_LEN) != 0)
        return;
 
    int idx = beacon_index(name);
    if (idx < 0)
        return;
 
    float rssi = (float)adv->getRssi();
    constexpr float MAX_RSSI_JUMP = 8.0f; // dB
    if (seen[idx] && fabsf(rssi - rssi_avg[idx]) > MAX_RSSI_JUMP)
    {
        // ignore this one sample, likely a multipath spike
        return;
    }
 
    if (!seen[idx])
    {
        rssi_avg[idx] = rssi;
        seen[idx] = true;
    }
    else
        rssi_avg[idx] = ALPHA * rssi + (1.0f - ALPHA) * rssi_avg[idx];
 
     Serial.print("r1="); Serial.print(rssi_avg[0], 1);
     Serial.print(" r2="); Serial.print(rssi_avg[1], 1);
     Serial.print(" r3="); Serial.println(rssi_avg[2], 1);
 
     Serial.print("d1="); Serial.print(rssi_to_distance(0, rssi_avg[0]), 1);
     Serial.print(" d2="); Serial.print(rssi_to_distance(1, rssi_avg[1]), 1);
     Serial.print(" d3="); Serial.println(rssi_to_distance(2, rssi_avg[2]), 1);
 
    float tx, ty;
    if (!trilaterate(&tx, &ty))
        return;
 
    kf.update(tx, ty);
 
    float fx = fmaxf(0.0f, fminf(100.0f, kf.x));
    float fy = fmaxf(0.0f, fminf(100.0f, kf.y));
 
     Serial.print("raw=(");
     Serial.print(tx, 1);
     Serial.print(",");
     Serial.print(ty, 1);
     Serial.print(")  kf=(");
     Serial.print(fx, 1);
     Serial.print(",");
     Serial.print(fy, 1);
     Serial.println(")");
        Serial.println(rssi);
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
constexpr float RIGHT_BASE_SPEED = 0.455f;
 
// Live straight-line correction
constexpr float ENCODER_KP = 0.45f;
constexpr float MAX_LIVE_CORRECTION = 0.10f;
 
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
 
RobotState robotState = GOING_OUT;
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
 
// =====================================================================
// Motor output
// =====================================================================
 
// =====================================================================
// Motor output (UNCHANGED, but kept for clarity)
// =====================================================================
 
void writeMotorCommands(
    float leftSpeed,
    float rightSpeed,
    int movementDirection
)
{
    leftSpeed = constrain(leftSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
 
    const int leftPulseUs =
        LEFT_STOP_US +
        static_cast<int>(
            LEFT_FORWARD_DIRECTION *
            movementDirection *
            leftSpeed *
            MAX_SERVO_OFFSET_US
        );
 
    const int rightPulseUs =
        RIGHT_STOP_US +
        static_cast<int>(
            RIGHT_FORWARD_DIRECTION *
            movementDirection *
            rightSpeed *
            MAX_SERVO_OFFSET_US
        );
 
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
            1
        );
 
        Serial.println("Motion mode: FORWARD");
    }
    else if (motionMode == MotionMode::BACKWARD)
{
    // use your normal speed system but adjust imbalance here
    writeMotorCommands(
        LEFT_BASE_SPEED  * 1.3f,
        RIGHT_BASE_SPEED * 0.30f,   // RIGHT WHEEL SLOWER ADJUST THIS
        -1
    );
 
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
        (nowMs - lastEncoderControlMs) <
        ENCODER_CONTROL_INTERVAL_MS
    )
    {
        return;
    }
 
    lastEncoderControlMs = nowMs;
 
    uint32_t currentLeftTicks;
    uint32_t currentRightTicks;
 
    readEncoderTicks(currentLeftTicks, currentRightTicks);
 
    const int32_t leftTickChange =
        static_cast<int32_t>(
            currentLeftTicks - previousLeftTicks
        );
 
    const int32_t rightTickChange =
        static_cast<int32_t>(
            currentRightTicks - previousRightTicks
        );
        Serial.print("mode=");
        Serial.print((motionMode == MotionMode::FORWARD) ? "FWD" :
             (motionMode == MotionMode::BACKWARD) ? "BWD" : "STOP");
 
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
                "No movement detected from the left wheel."
            );
        }
 
        if (rightEncoderTimedOut)
        {
            Serial.println(
                "No movement detected from the right wheel."
            );
        }
 
        robotState = HOME;
        setMotionMode(MotionMode::STOPPED);
        return;
    }
 
    const float totalMovement =
        static_cast<float>(
            leftTickChange + rightTickChange
        );
 
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
            leftTickChange - rightTickChange
        ) / totalMovement;
 
    float liveCorrection =
        ENCODER_KP * normalizedError;
 
    liveCorrection = constrain(
        liveCorrection,
        -MAX_LIVE_CORRECTION,
        MAX_LIVE_CORRECTION
    );
 
    const float leftCommand =
        LEFT_BASE_SPEED + liveCorrection;
 
    const float rightCommand =
        RIGHT_BASE_SPEED - liveCorrection;
 
    const int movementDirection =
        motionMode == MotionMode::FORWARD
        ? 1
        : -1;
 
    writeMotorCommands(
        leftCommand,
        rightCommand,
        movementDirection
    );
 
    //  Serial.print("Encoder L=");
    //  Serial.print(currentLeftTicks);
 
    //  Serial.print(" R=");
    //  Serial.print(currentRightTicks);
 
    //  Serial.print(" | dL=");
    //  Serial.print(leftTickChange);
 
    //  Serial.print(" dR=");
    //  Serial.print(rightTickChange);
 
    //  Serial.print(" | error=");
    //  Serial.print(normalizedError, 4);
 
    //  Serial.print(" | correction=");
    //  Serial.print(liveCorrection, 4);
 
    //  Serial.print(" | motorL=");
    //  Serial.print(leftCommand, 4);
 
    //  Serial.print(" motorR=");
    //  Serial.println(rightCommand, 4);
}
 
// =====================================================================
// Hardware setup
// =====================================================================
 
void setupMotors()
{
    leftMotor.attach(
        LEFT_MOTOR_PIN,
        1000,
        2000
    );
 
    rightMotor.attach(
        RIGHT_MOTOR_PIN,
        1000,
        2000
    );
 
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
        FALLING
    );
 
    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        rightEncoderInterrupt,
        FALLING
    );
 
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
        // Detect obstacle only once
        if (!pushingObstacle && distance < STOP_DISTANCE_MM)
        {
            pushingObstacle = true;
            pushStartTime = millis();
 
            Serial.println("Obstacle detected. Pushing...");
        }
 
        // After 3 seconds of pushing, go home
        if (pushingObstacle &&
            (millis() - pushStartTime >= 10000))
        {
            readEncoderTicks(forwardLeftTicks, forwardRightTicks);
 
            reverseStartLeftTicks = leftEncoderTicks;
            reverseStartRightTicks = rightEncoderTicks;
 
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
 
        uint32_t leftBack  = leftTicks  - reverseStartLeftTicks;
        uint32_t rightBack = rightTicks - reverseStartRightTicks;
 
        if (leftBack >= forwardLeftTicks &&
            rightBack >= forwardRightTicks)
        {
            robotState = HOME;
            pushingObstacle = false;
 
            Serial.println("Returned to start.");
 
            setMotionMode(MotionMode::STOPPED);
        }
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
        "Starting BLE positioning, ToF and encoder correction"
    );
 
    setupMotors();
    setupEncoders();
    setupToF();
    setMotionMode(MotionMode::FORWARD);
 
    kf.initialised = false;
 
    BTstack.setBLEAdvertisementCallback(
        advertisementCallback
    );
 
    BTstack.setup();
    BTstack.bleStartScanning();
 
    Serial.println("Pico is scanning and driving.");
}
 
void loop()
{
    BTstack.loop();
    handleObstacleAvoidance();
    updateWheelCorrection();
}
 
