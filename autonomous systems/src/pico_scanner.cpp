#include <Arduino.h>    // using Arduino framework for setup and loop functions
#include <BTstackLib.h> // using BTstack as the Bluetooth stack implementation
#include <SPI.h>        // required by BTstack for SPI communication with the Bluetooth controller

#define BEACON_PREFIX "MyBeacon"
#define BEACON_PREFIX_LEN (sizeof(BEACON_PREFIX) - 1) // length of the beacon prefix string, excluding the null terminator

static bool parse_name(const uint8_t *data, uint8_t max_len, char *out, uint8_t out_size) // parse the device name from the BLE advertisement data, returns true if a name was found and copied to out
{
    int i = 0;
    while (i < max_len) // loop through advertisement data chunks until find a name or reach the end of the data
    {
        uint8_t chunk_len = data[i];
        if (chunk_len == 0)
            break;
        if (i + chunk_len > max_len)
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
        i += 1 + chunk_len; // move to the next chunk (1 byte for length + chunk_len bytes for data)
    }
    return false;
}

void advertisementCallback(BLEAdvertisement *adv) // callback function that is called by BTstack for each received BLE advertisement, extracts and prints the device name and RSSI
{
    const uint8_t *data = adv->getAdvData(); // get the raw advertisement data from the BLEAdvertisement object

    char name[32] = {0};
    if (!parse_name(data, 31, name, sizeof(name)))
        return;

    if (strncmp(name, BEACON_PREFIX, BEACON_PREFIX_LEN) != 0) // check if the device name starts with the expected beacon prefix, if not ignore this advertisement
        return;
    
    Serial.print("Beacon found: ");
    Serial.print(name);
    Serial.print("  RSSI: ");
    Serial.println(adv->getRssi());
}

void setup() 
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("Pico W BLE Scanner");

    BTstack.setBLEAdvertisementCallback(advertisementCallback);
    BTstack.setup();
    BTstack.bleStartScanning();
}

void loop()
{
    BTstack.loop();
}