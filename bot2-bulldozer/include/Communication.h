#pragma once

#include <WiFi.h>
#include <PubSubClient.h>

extern const char *ssid;
extern const char *password;
extern const char *mqttServer;
extern const char *COMMAND_TOPIC;

extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

void connectWiFi();
void connectMQTT();

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length);
