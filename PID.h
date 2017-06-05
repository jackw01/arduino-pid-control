//
// PID control implementation for Arduino and other AVR processors programmable with the Arduino IDE
// Copyright (c) 2017 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details
//

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {

    public:

        PID();
        PID(double p, double i, double d, double min, double max);

        double getSetpoint(); // Get setpoint
        void setSetpoint(double set); // Set setpoint
        void setOutputRange(double min, double max); // Set output range
        void setPID(double p, double i, double d); // Set PID gains
        void setDerivativeOnMeasurement(bool on); // Enable or disable derivative on measurement (reduces spikes when setpoint changes)

        // Calculate the output based on input
        // This method should be called in a loop at a regular interval
        // (faster loops are required for faster-changing inputs)
        double calculate(double input);

        void reset(); // Reset the controller

    private:

        // Setpoint
        double setpoint = 0;

        // Range
        double minOutput;
        double maxOutput;

        // PID constants
        double kp;
        double ki;
        double kd;

        // Loop variables
        double error = 0;
        double totalError = 0;
        double prevInput = 0;

        // Other variables
        bool derivativeOnMeasurement = false;
};

#endif
