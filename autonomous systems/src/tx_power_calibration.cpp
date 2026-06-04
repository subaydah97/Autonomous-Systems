#include <Arduino.h>
#include <BTstackLib.h>
#include <SPI.h>

// change based on where pico is and how big field is 
const float PICO_X = 49.0f; 
const float PICO_Y = 49.0f; 

const float BX[3] = {0.0f, 0.0f, 98.0f};   
const float BY[3] = {0.0f, 98.0f, 98.0f};  
const float n = 2.5f;                       

#define BEACON_PREFIX "ForestBeacon"
#define NUM_SAMPLES 50  

// calibrated 1-unit baseline transmission powers
float rssi_sum[3] = {0.0f, 0.0f, 0.0f};
int rssi_count[3] = {0, 0, 0};
bool calibration_done = false;

static int beacon_index(const char *name) { // returns the index of the beacon based on its name, or -1 if the name doesn't match any known beacons
    if (strcmp(name, "ForestBeaconOne") == 0)   return 0;
    if (strcmp(name, "ForestBeaconTwo") == 0)   return 1;
    if (strcmp(name, "ForestBeaconThree") == 0) return 2;
    return -1;
}

static bool parse_name(const uint8_t *data, uint8_t max_len, char *out, uint8_t out_size) {
    int i = 0;
    while (i < max_len) { // loop through the advertisement data chunks to find the one containing the device name
        uint8_t chunk_len = data[i];
        if (chunk_len == 0 || i + chunk_len > max_len) break;
        uint8_t chunk_type = data[i + 1];
        if (chunk_type == 0x09 || chunk_type == 0x08) {
            uint8_t name_len = chunk_len - 1;
            if (name_len >= out_size) name_len = out_size - 1;
            memcpy(out, &data[i + 2], name_len);
            out[name_len] = '\0';
            return true;
        }
        i += 1 + chunk_len;
    }
    return false;
}

// converts current RSSI to an estimated distance based on calibrated parameters
void advertisementCallback(BLEAdvertisement *adv) {
    if (calibration_done) return;

    char name[32] = {0};
    if (!parse_name(adv->getAdvData(), 31, name, sizeof(name))) return;
    
    int idx = beacon_index(name);
    if (idx < 0) return; 

    if (rssi_count[idx] < NUM_SAMPLES) {
        rssi_sum[idx] += (float)adv->getRssi();
        rssi_count[idx]++;
        
        Serial.print("sampling b"); Serial.print(idx);
        Serial.print(": "); Serial.print(rssi_count[idx]);
        Serial.print("/"); Serial.println(NUM_SAMPLES);
    }

    // if all samples are collected, calculate the average rssi for each beacon
    if (rssi_count[0] >= NUM_SAMPLES && rssi_count[1] >= NUM_SAMPLES && rssi_count[2] >= NUM_SAMPLES) {
        calibration_done = true;
        BTstack.bleStopScanning(); 

        float final_tx_power[3];
        Serial.println("\ncalibration done:");

        for (int i = 0; i < 3; i++) {
            float avg_rssi = rssi_sum[i] / NUM_SAMPLES;
            float dx = PICO_X - BX[i];
            float dy = PICO_Y - BY[i];
            float true_distance = sqrtf(dx * dx + dy * dy);

            final_tx_power[i] = avg_rssi + (10.0f * n * log10f(true_distance));
        }

        Serial.print("const float TX_POWER[3] = {"); // paste into pico_scanner.cpp
        Serial.print(final_tx_power[0], 2); Serial.print("f, ");
        Serial.print(final_tx_power[1], 2); Serial.print("f, ");
        Serial.print(final_tx_power[2], 2); Serial.println("f};");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("starting calibration...");
    
    BTstack.setBLEAdvertisementCallback(advertisementCallback);
    BTstack.setup();
    BTstack.bleStartScanning();
}

void loop() {
    BTstack.loop();
}