/*
 * Telemetry.h
 *
 * Handles communication between the Scout Chariot and the MQTT network.
 *
 * This header declares the functions responsible for publishing
 * detected obstacle information and transmitting the robot's
 * telemetry data, such as its position and wheel encoder status.
 */

#pragma once

// Publishes detected obstacle coordinates to the MQTT broker.
void handleObstaclePublish();

// Publishes the robot's telemetry data.
void handleTelemetry();
