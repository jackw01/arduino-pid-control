// PID control implementation for Arduino and other AVR processors programmable with the Arduino IDE
// Copyright (c) 2016-2018 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#include "pidcontroller.h"

PID::PID() {
}

PID::PID(float p, float i, float d, float min, float max) {
  setPID(p, i, d);
  setOutputRange(min, max);
}

// Get setpoint
float PID::getSetpoint() {
  return setpoint;
}

// Set setpoint
void PID::setSetpoint(float set) {
  setpoint = set;
}

// Set output range
void PID::setOutputRange(float min, float max) {
  minOutput = min;
  maxOutput = max;
}

// Set PID gains
void PID::setPID(float p, float i, float d) {
  kp = p;
  ki = i;
  kd = d;
}

// Calculate the output based on input
float PID::calculate(float input) {
  float prevError = error;
  error = setpoint - input;
  if ((error * kp < maxOutput) && (error * kp > minOutput)) totalError += error;
  else totalError = 0;
  float dTerm = (error - prevError);
  prevInput = input;
  float result = kp * error + ki * totalError - kd * dTerm;
  if (result > maxOutput) result = maxOutput;
  else if (result < minOutput) result = minOutput;
  return result;
}

// Reset the controller
void PID::reset() {
  error = 0;
  totalError = 0;
}
