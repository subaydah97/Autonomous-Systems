#include <cstring>
#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"

// Wi-Fi credentials and MQTT configuration.
const char *ssid = "Flowering Azalea Leaves";
const char *password = "ALOHAnet";
const char *mqttServer = "172.20.10.11";
const char *COMMAND_TOPIC = "bot/1/COMMANDS";

// Shared Wi-Fi and MQTT client instances.
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Connects the Pico W to the configured Wi-Fi network.
void connectWiFi()
{
    Serial.print("Connecting to WiFi");

    WiFi.begin(ssid, password);

    // Wait until the Wi-Fi connection has been established.
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

// Connects to the MQTT broker and subscribes to the command topic.
void connectMQTT()
{
    mqttClient.setServer(mqttServer, 1883);

    // Keep trying until a connection to the broker is made.
    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");

        if (mqttClient.connect("PicoW"))
        {
            Serial.println("connected");

            // Subscribe to receive commands for this robot.
            mqttClient.subscribe(COMMAND_TOPIC);
            Serial.println("Subscribed to bot/1/COMMANDS");
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

// Handles incoming MQTT messages.
void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
    // Ignore messages from other topics.
    if (strcmp(topic, COMMAND_TOPIC) != 0)
    {
        return;
    }

    // Convert the received payload into a String.
    String command;
    command.reserve(length);

    for (unsigned int i = 0; i < length; i++)
    {
        command += static_cast<char>(payload[i]);
    }

    command.trim();

    Serial.print("Command received: ");
    Serial.println(command);

    // Activate the emergency stop when the ESTOP command is received.
    if (command == "ESTOP")
    {
        activateEmergencyStop();
    }
}
