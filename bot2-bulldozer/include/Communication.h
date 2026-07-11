/*
 * Communication.h
 *
 * Handles all network communication for Bot 2 (Bulldozer Chariot).
 *
 * This header declares the Wi-Fi and MQTT configuration used to
 * connect the robot to the local network and MQTT broker. It also
 * exposes the shared MQTT client, connection functions,
 */

#pragma once

#include <WiFi.h>
#include <PubSubClient.h>

// Contains the Wi-Fi and MQTT configuration used by the
// bulldozer to communicate with the MQTT broker.

extern const char *ssid;
extern const char *password;
extern const char *mqttServer;

// MQTT topic used to receive obstacle coordinates from
// the obstacle registry.
extern const char *OBSTACLE_TOPIC;

// MQTT topic used to receive control commands.
extern const char *COMMAND_TOPIC;

// Global Wi-Fi and MQTT client objects.
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

// Connects the robot to the configured Wi-Fi network.
void connectWiFi();

// Connects to the MQTT broker and subscribes to the
// required MQTT topics.
void connectMQTT();
