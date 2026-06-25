#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
 
// ToF pins
#define TOF_SDA_PIN 4
#define TOF_SCL_PIN 5
 
// Servo pins
#define LEFT_MOTOR_PIN 6    // PWM_LM
#define RIGHT_MOTOR_PIN 7   // PWM_RM
 
// Obstacle logic
#define STOP_DISTANCE_MM 50
#define RESUME_DISTANCE_MM 60
 
// MicroPython-style PWM values
// 5000 = stop
// Voor achteruit: links onder 5000, rechts boven 5000
#define MOTOR_STOP_U16 5000
#define LEFT_BACKWARD_U16 3800
#define RIGHT_BACKWARD_U16 6200
 
VL53L1X sensor;
 
Servo leftMotor;
Servo rightMotor;
 
bool motorsStopped = true;
 
int dutyU16ToMicroseconds(uint16_t duty_u16) {
  return (int)((duty_u16 / 65535.0f) * 20000.0f);
}
 
void setMotorDutyU16(Servo &motor, uint16_t duty_u16) {
  int pulse_us = dutyU16ToMicroseconds(duty_u16);
  motor.writeMicroseconds(pulse_us);
}
 
void stopMotors() {
  setMotorDutyU16(leftMotor, MOTOR_STOP_U16);
  setMotorDutyU16(rightMotor, MOTOR_STOP_U16);
}
 
void driveBackward() {
  setMotorDutyU16(leftMotor, LEFT_BACKWARD_U16);
  setMotorDutyU16(rightMotor, RIGHT_BACKWARD_U16);
}
 
void setupMotors() {
  leftMotor.attach(LEFT_MOTOR_PIN, 500, 2500);
  rightMotor.attach(RIGHT_MOTOR_PIN, 500, 2500);
 
  stopMotors();
  delay(1000);
}
 
void setupToF() {
  Wire.begin();
  Wire.setClock(100000);
 
  sensor.setTimeout(500);
 
  if (!sensor.init()) {
    Serial.println("VL53L1X niet gevonden");
 
    while (true) {
      stopMotors();
      delay(1000);
    }
  }
 
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(100);
}
 
void setup() {
  Serial.begin(115200);
  delay(2000);
 
  Serial.println("Starting ToF + Servo control");
 
  setupMotors();
  setupToF();
 
  Serial.println("tijd_ms : afstand_mm : status : motor_state : left_u16 : right_u16 : left_us : right_us");
}
 
void loop() {
  uint16_t distance = sensor.read();
 
  if (sensor.timeoutOccurred()) {
    stopMotors();
    motorsStopped = true;
 
    Serial.print(millis());
    Serial.print(" : ");
    Serial.print(distance);
    Serial.print(" : timeout : STOPPED : ");
    Serial.print(MOTOR_STOP_U16);
    Serial.print(" : ");
    Serial.print(MOTOR_STOP_U16);
    Serial.print(" : ");
    Serial.print(dutyU16ToMicroseconds(MOTOR_STOP_U16));
    Serial.print(" : ");
    Serial.println(dutyU16ToMicroseconds(MOTOR_STOP_U16));
 
    delay(100);
    return;
  }
 
  if (distance < STOP_DISTANCE_MM) {
    stopMotors();
    motorsStopped = true;
  }
  else if (distance > RESUME_DISTANCE_MM) {
    driveBackward();
    motorsStopped = false;
  }
 
  Serial.print(millis());
  Serial.print(" : ");
  Serial.print(distance);
  Serial.print(" : ok : ");
 
  if (motorsStopped) {
    Serial.print("STOPPED : ");
    Serial.print(MOTOR_STOP_U16);
    Serial.print(" : ");
    Serial.print(MOTOR_STOP_U16);
    Serial.print(" : ");
    Serial.print(dutyU16ToMicroseconds(MOTOR_STOP_U16));
    Serial.print(" : ");
    Serial.println(dutyU16ToMicroseconds(MOTOR_STOP_U16));
  } else {
    Serial.print("DRIVING_BACKWARD : ");
    Serial.print(LEFT_BACKWARD_U16);
    Serial.print(" : ");
    Serial.print(RIGHT_BACKWARD_U16);
    Serial.print(" : ");
    Serial.print(dutyU16ToMicroseconds(LEFT_BACKWARD_U16));
    Serial.print(" : ");
    Serial.println(dutyU16ToMicroseconds(RIGHT_BACKWARD_U16));
  }
 
  delay(100);
}