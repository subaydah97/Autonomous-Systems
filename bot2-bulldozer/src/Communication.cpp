#include <cstring>
#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"

// Defines the network connection details used by the
// bulldozer to communicate with the MQTT broker.
//
// The bulldozer receives obstacle locations from the
// obstacle registry and listens for control commands.

const char *ssid = "Flowering Azalea Leaves";
const char *password = "ALOHAnet";
const char *mqttServer = "172.20.10.11";

// MQTT topic used to receive direct commands.
const char *COMMAND_TOPIC = "bot/2/COMMANDS";

// MQTT topic used to receive new obstacle coordinates.
const char *OBSTACLE_TOPIC = "OR/NEW";

// Global communication objects.
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Connects the bulldozer to the configured Wi-Fi network.
// The robot waits here until a connection is established.
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

    // Display the assigned IP address for debugging.
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}
// Connects to the MQTT broker and subscribes to the
// topics required by the bulldozer.
//
// The MQTT client name is unique so it does not conflict
// with the scout chariot connection.
void connectMQTT()
{
    mqttClient.setServer(mqttServer, 1883);

    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");

        // Unique client ID for Bot 2.
        if (mqttClient.connect("PicoW_Bulldozer"))
        {
            Serial.println("connected");

            // Receive obstacle locations and commands.
            mqttClient.subscribe(COMMAND_TOPIC);
            mqttClient.subscribe(OBSTACLE_TOPIC);

            Serial.println("Subscribed to OR/NEW");
            Serial.println("Subscribed to bot/2/COMMANDS");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying...");

            // Retry connection after a short delay.
            delay(2000);
        }
    }
}
