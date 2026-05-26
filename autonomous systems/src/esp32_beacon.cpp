#include <Arduino.h>
#include <NimBLEDevice.h> // using NimBLE-Arduino library for BLE functionality

#define BEACON_NAME "MyBeacon" // name for the esp beacon add 1 2 3 when using other beacons

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32 BLE Beacon starting...");

    NimBLEDevice::init(BEACON_NAME); // init nimble stack with the device name

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising(); // create advertising object and set params

    pAdvertising->setName(BEACON_NAME); // set the name of the device to be advertised

    pAdvertising->setAdvertisementType(BLE_GAP_CONN_MODE_NON); // set the advertising type to non-connectable (beacon mode)

    pAdvertising->start(); // start advertising

    Serial.print("Broadcasting as: ");
    Serial.println(BEACON_NAME);
}

void loop()
{
    delay(1000); // do nothing, the beacon is advertising in the background
}