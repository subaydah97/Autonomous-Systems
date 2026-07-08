#pragma once

#include <Arduino.h>

void handleObstacleAvoidance();
void mqttCallback(char *topic, byte *payload, unsigned int length);
