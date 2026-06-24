#include <Arduino.h>    // using Arduino framework for setup and loop functions
#include <BTstackLib.h> // using BTstack as the Bluetooth stack implementation
#include <SPI.h>        // required by BTstack for SPI communication with the Bluetooth controller

#define BEACON_PREFIX "ForestBeacon"                  // prefix for the beacon names to filter in the scanner, should match the names used by the ESP32 beacons
#define BEACON_PREFIX_LEN (sizeof(BEACON_PREFIX) - 1) // length of the beacon prefix string, excluding the null terminator

// beacon `coordinates` in the environment, used for later localization calculations
const float BX[3] = {0.0f, 100.0f, 50.0f};   
const float BY[3] = {0.0f, 0.0f, 100.0f}; 

// calibrated 1-unit baseline transmission powers
// these values allow the log-distance path loss equation to calculate the correct distance 
const float TX_POWER[3] = {-19.78f, -12.44f, -12.93f};//use calibration code from tx_power_calibration.cpp to find these values 
const float ALPHA = 0.08f;                        // smoothing factor for the exponential moving average filter applied to RSSI values, decrease if the position jumps too much, increase if it lags behind
const float n = 2.5f;                             // path loss exponent for distance calculation
float rssi_avg[3] = {-100.0f, -100.0f, -100.0f}; // array to store the smoothed RSSI values for each beacon
bool seen[3] = {false, false, false};            // array to track whether each beacon has been seen at least once

struct Kalman2D
{
    float x, y, vx, vy; // state vector: position (x,y) and velocity (vx,vy)
    float P[4][4];      // covariance matrix representing the uncertainty of the state estimate

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

        Q_pos = 0.1f; // decrease if position jumps too much, decrease if it lags
        Q_vel = 0.1f;
        R = 80.0f; 

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

        // lalman gain
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

// converts current RSSI to an estimated distance based on calibrated parameters
static float rssi_to_distance(int idx, float rssi)
{
    return powf(10.0f, (TX_POWER[idx] - rssi) / (10.0f * n)); 
}

static bool trilaterate(float *out_x, float *out_y)
{
    // convert smoothed RSSI values to distance estimates
    float d0 = rssi_to_distance(0, rssi_avg[0]);
    float d1 = rssi_to_distance(1, rssi_avg[1]);
    float d2 = rssi_to_distance(2, rssi_avg[2]);

    float x1 = BX[0], y1 = BY[0], x2 = BX[1], y2 = BY[1], x3 = BX[2], y3 = BY[2]; // beacon coordinates

    // trilateration math to solve for (x,y) given the three circles defined by (x1,y1,d0), (x2,y2,d1), (x3,y3,d2)
    float A = 2 * (x2 - x1);
    float B = 2 * (y2 - y1);
    float C = d0 * d0 - d1 * d1 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
    float D = 2 * (x3 - x1);
    float E = 2 * (y3 - y1);
    float F = d0 * d0 - d2 * d2 - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;

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
    while (i < max_len) // loop through the advertisement data chunks to find the one containing the device name 
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
    if (!seen[idx])
    {
        rssi_avg[idx] = rssi;
        seen[idx] = true;
    }
    else
        rssi_avg[idx] = ALPHA * rssi + (1.0f - ALPHA) * rssi_avg[idx];

    if (!seen[0] || !seen[1] || !seen[2])
        return;

    // debug prints to monitor the RSSI values, distance estimates, and final position estimates    
    Serial.print("r0=");   Serial.print(rssi_avg[0], 1);
    Serial.print(" r1=");  Serial.print(rssi_avg[1], 1);
    Serial.print(" r2=");  Serial.println(rssi_avg[2], 1);

    Serial.print("d0=");   Serial.print(rssi_to_distance(0, rssi_avg[0]), 1);
    Serial.print(" d1=");  Serial.print(rssi_to_distance(1, rssi_avg[1]), 1);
    Serial.print(" d2=");  Serial.println(rssi_to_distance(2, rssi_avg[2]), 1);

    // perform trilateration to estimate the (x,y) position based on the distances to the three beacons
    float tx, ty;
    if (!trilaterate(&tx, &ty))
        return;

    // feed raw trilaterated position into kalman filter
    kf.update(tx, ty);

    // clamp inside your physical boundary lines and print
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
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    kf.initialised = false; // ensure kalman filter starts uninitialised so it can set its initial state on the first measurement
    Serial.println("pico is scanning..");
    BTstack.setBLEAdvertisementCallback(advertisementCallback);
    BTstack.setup();
    BTstack.bleStartScanning();
}

void loop()
{
    BTstack.loop();
}