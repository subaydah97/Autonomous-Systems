#include <Arduino.h>
#include "Communication.h"

const char *ssid = "mn-92937";
const char *password = "12345679";
const char *mqttServer = "192.168.212.106";

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
