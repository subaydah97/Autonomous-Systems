/*
 * ObstacleHandler.h
 *
 * Manages obstacle detection and avoidance for the Scout Chariot.
 *
 * This header declares the function responsible for monitoring
 * sensor data and updating the robot's behaviour when obstacles
 * are detected during navigation.
 */

#pragma once

#include <Arduino.h>
// Main obstacle handling routine
void mqttCallback(char *topic, byte *payload, unsigned int length);
