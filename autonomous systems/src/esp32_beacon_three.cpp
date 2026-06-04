#include <Arduino.h>
#include <NimBLEDevice.h> // using NimBLE-Arduino library for BLE functionality

#define BEACON_NAME "ForestBeaconThree" // name for the esp beacon add 1 2 3 when using other beacons
#define LED_PIN 2              // built-in LED pin for ESP32, will blink to indicate status

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    delay(1000);
    Serial.println("ESP32 BLE Beacon starting...");

    NimBLEDevice::init(BEACON_NAME); // init nimble stack with the device name

    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // set maximum power level for strong signal)

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising(); // create advertising object and set params

    pAdvertising->setName(BEACON_NAME); // set the name of the device to be advertised

    pAdvertising->setAdvertisementType(BLE_GAP_CONN_MODE_NON); // set the advertising type to non-connectable (beacon mode)

    bool started = pAdvertising->start(0); // start advertising with no timeout (0 means advertise indefinitely until stop is called)

    if (started)
    {
        // 3 fast blinks = broadcasting
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
        Serial.print("Broadcasting as: ");
        Serial.println(BEACON_NAME);
    }
    else
    {
        // 10 rapid blinks = failed to start
        for (int i = 0; i < 10; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
        Serial.println("Failed to start advertising!");
    }
}

void loop()
{
    // slow blink = alive and broadcasting
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}