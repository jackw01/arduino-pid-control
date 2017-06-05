//
// Copyright (c) 2016-2017 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details
//

#include "PID.h"

PID::PID() {

}

PID::PID(double p, double i, double d, double min, double max) {

    setPID(p, i, d);
    setOutputRange(min, max);
}

// Get setpoint
double PID::getSetpoint() {

    return setpoint;
}

// Set setpoint
void PID::setSetpoint(double set) {

    setpoint = set;
}

// Set output range
void PID::setOutputRange(double min, double max) {

    minOutput = min;
    maxOutput = max;
}

// Set PID gains
void PID::setPID(double p, double i, double d) {

    kp = p;
    ki = i;
    kd = d;
}

void PID::setDerivativeOnMeasurement(bool on) {

    derivativeOnMeasurement = on;
}

// Calculate the output based on input
double PID::calculate(double input) {

    double prevError = error;
    error = setpoint - input;
    totalError += error;
    double dTerm = ((derivativeOnMeasurement) ? input - prevInput : error - prevError);
    prevInput = input;
    return constrain(kp * error + ki * totalError - kd * dTerm, minOutput, maxOutput);
}

// Reset the controller
void PID::reset() {

    error = 0;
    totalError = 0;
}
