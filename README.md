# Temp-O-Matic
An Arduino-based thermostat controlling 12V output through PWM
-----------------------------------------------------------------

The thermostat outputs 0-VIN based on setpoint and measured temperature.
It linearly increases the PWM to the MOSFET (ergo, increases the output-voltage) between the setpoint and setpoint + diff.

The setpoint is changed by ways of a rotary-encoder, and is displayed on a 7-segment 4-digit TM1637 based display. Setpoint is shown as the first two digits, and measured temperature is displayed as the two rightmost digits.

Fine-tuning params of note are the constants:

const long interval = 1000; // Time between temperature polls.
const int diff = 2; // Hysteresis for temperature curve.
const int minFan = 45; // Lowest speed the fan starts at.
const int lcdBrightness = 20; // Sets the lcd brightness.

and the setPwmFrequency(10, 1024); function calls which changes the PWM freq to avoid noise from the motor.


Changelog:

Version 1.0.1 - Initial commit.
