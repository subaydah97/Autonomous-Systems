/*
 * Communication.h
 *
 * Handles all network communication for Bot 1 (Scout Chariot).
 *
 * This header declares the Wi-Fi and MQTT configuration used to
 * connect the robot to the local network and MQTT broker. It also
 * exposes the shared MQTT client, connection functions, and the
 * callback that processes incoming MQTT messages.
 */

#pragma once

#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi and MQTT configuration
extern const char *ssid;
extern const char *password;
extern const char *mqttServer;
extern const char *COMMAND_TOPIC;

// Shared network clients
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

// Establish Wi-Fi and MQTT connections
void connectWiFi();
void connectMQTT();

// Handles incoming MQTT messages
void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length);
