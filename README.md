# arduino-pid-control
Versatile PID control library for Arduino and Arduino-compatible microcontrollers

## Example Code
```arduino
#include <PID.h>

void setup() {

    // kp, ki, and kd are tuning constants
    // minOutput and maxOutput define the output range
    // should be 0 and 255 for analogWrite
    // output is a double
    PID pidController = PID(kp, ki, kd, minOutput, maxOutput);
}

void loop() {

    long startTime = micros();

    // Get input (value from temperature sensor, potentiometer value, encoder position, etc.)
    double output = pidController.calculate(input);
    // Do things with output (usually analogWrite)

    // Waste time until the loop is ready to run again
    // pidController.calculate() needs to be called at a regular interval
    // smaller intervals are necessary for faster-changing input signals
    // usually should be <= 1 second for temperature and <= 10 milliseconds for motor/encoder setups
    // faster is not always better - it is possible for the loop to be too fast
    while (micros() - startTime < loopInterval) delayMicroseconds(1);
}
```
