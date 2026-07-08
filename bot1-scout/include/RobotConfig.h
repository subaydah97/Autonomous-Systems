#pragma once

#include <Arduino.h>

#define _telemetry_target "bot/1"

// ToF pins
#define TOF_SDA_PIN 4
#define TOF_SCL_PIN 5

// Motor pins
constexpr uint8_t LEFT_MOTOR_PIN = 6;
constexpr uint8_t RIGHT_MOTOR_PIN = 7;

// IR encoder output pins
constexpr uint8_t LEFT_ENCODER_PIN = 11;
constexpr uint8_t RIGHT_ENCODER_PIN = 14;

// Obstacle handling
constexpr uint16_t STOP_DISTANCE_MM = 50;

// Continuous-rotation servo calibration
constexpr int LEFT_STOP_US = 1500;
constexpr int RIGHT_STOP_US = 1500;

// These signs must make both wheels move forward for positive commands.
constexpr int LEFT_FORWARD_DIRECTION = 1;
constexpr int RIGHT_FORWARD_DIRECTION = -1;

constexpr int MAX_SERVO_OFFSET_US = 250;

// Calibrated base commands from the encoder test
constexpr float LEFT_BASE_SPEED = 0.418f;
constexpr float RIGHT_BASE_SPEED = 0.500f;

// Live straight-line correction
constexpr float ENCODER_KP = 0.25f;
constexpr float MAX_LIVE_CORRECTION = 0.06f;
constexpr float LEFT_CORRECTION_SCALE = 0.20f;
constexpr float RIGHT_CORRECTION_SCALE = 1.5f;

constexpr float MIN_DRIVE_SPEED = 0.30f;
constexpr float MAX_DRIVE_SPEED = 0.65f;

// The sensors produce only a few ticks per second, so one second
// provides a more useful comparison than a very short interval.
constexpr uint32_t ENCODER_CONTROL_INTERVAL_MS = 500;
constexpr uint32_t ENCODER_TIMEOUT_MS = 2500;
constexpr uint32_t MIN_ENCODER_INTERVAL_US = 3000;
constexpr float WHEEL_TICK_TO_RAD = (2.0f * PI) / 10.0f;
constexpr float TICK_TO_CM = 0.72f * PI; // 1 tick ≈ 2.2619 cm (0.442 tick = 1 cm)

const int TELEMETRY_INTERVAL_MS = 1000 / 2;
