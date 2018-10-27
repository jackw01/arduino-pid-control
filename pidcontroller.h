// PID control implementation for Arduino and other AVR processors programmable with the Arduino IDE
// Copyright (c) 2016-2018 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#ifndef PID_H
#define PID_H

class PID {
public:
  PID();
  PID(float p, float i, float d, float min, float max);

  float getSetpoint(); // Get setpoint
  void setSetpoint(float set); // Set setpoint
  void setOutputRange(float min, float max); // Set output range
  void setPID(float p, float i, float d); // Set PID gains

  // Calculate the output based on input
  // This method should be called in a loop at a regular interval
  // (faster loops are required for faster-changing inputs)
  float calculate(float input);

  void reset(); // Reset the controller

private:
  // Setpoint
  float setpoint = 0;

  // Range
  float minOutput;
  float maxOutput;

  // PID constants
  float kp;
  float ki;
  float kd;

  // Loop variables
  float error = 0;
  float totalError = 0;
  float prevInput = 0;
};

#endif
