#include <cstring>
#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"

const char *ssid = "mn-92937";
const char *password = "12345679";
const char *mqttServer = "192.168.212.106";
const char *COMMAND_TOPIC = "bot/1/COMMANDS";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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

        if (mqttClient.connect("PicoW"))
        {
            Serial.println("connected");
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

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
    if (strcmp(topic, COMMAND_TOPIC) != 0)
    {
        return;
    }

    String command;
    command.reserve(length);

    for (unsigned int i = 0; i < length; i++)
    {
        command += static_cast<char>(payload[i]);
    }

    command.trim();

    Serial.print("Command received: ");
    Serial.println(command);

    if (command == "ESTOP")
    {
        activateEmergencyStop();
    }
}
