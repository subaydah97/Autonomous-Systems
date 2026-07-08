#include <cstring>
#include <Arduino.h>
#include "Communication.h"
#include "DriveSystem.h"

const char *ssid = "mn-92937";
const char *password = "12345679";
const char *mqttServer = "192.168.212.106";
const char *COMMAND_TOPIC = "bot/2/COMMANDS";
const char *OBSTACLE_TOPIC = "OR/NEW";

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

        // Must be different from bot 1
        if (mqttClient.connect("PicoW_Bulldozer"))
        {
            Serial.println("connected");

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

            delay(2000);
        }
    }
}
