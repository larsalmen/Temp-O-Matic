/**********************************************************************************************************
    Name    : Tempomatic
    Author  : Lars Almén
    Created : 2017-11-21
    Last Modified: 2018-07-17
    Version : 1.0.1
    Notes   : A simple thermostat. It controls a PWM output based on the temperature readings from a Dallas DS18B20 and a setpoint + diff.
              The setpoint and current temp is visualised on a 4-digit alpha-numerical display, and a rotary encoder is used to set the setpoint.
    License : This software is available under MIT License.
              Copyright 2017 Lars Almén
              Permission is hereby granted, free of charge, to any person obtaining a copy
              of this software and associated documentation files (the "Software"),
              to deal in the Software without restriction, including without limitation the rights to use,
              copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
              and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
              The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
              THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
              INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
              IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
              WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ***********************************************************************************************************/

// Some includes.
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SevenSegmentTM1637.h>
/********************************************************************/
// Define the pins.
#define EncOutputA 2
#define EncOutputB 4
#define ONE_WIRE_BUS 7
#define DIO 8
#define CLK 9
#define PWM_Control 10
/********************************************************************/

OneWire oneWire(ONE_WIRE_BUS); // Initialize the one-wire bus.
DallasTemperature sensors(&oneWire); // Pass the bus to the DallasTemp lib.
SevenSegmentTM1637 display(CLK, DIO);  // Set up the 4-Digit Display.

// Constants
const long interval = 1000; // Time between temperature polls.
const int diff = 2; // Hysteresis for temperature curve.
const int minFan = 45; // Lowest speed the fan starts at.
const int lcdBrightness = 20; // Sets the lcd brightness.

// Temp variables
float temp = 0;
volatile int setpoint = 22; // Volatile to make sure it is accessable inside the interrupt funcs.
int pwmValue = 0; // Output to fan.
int percentage = 0; // Calculated desired output to fan in percentage.
// Encoder variables
volatile int aState; // Current enc state.
volatile int aLastState; // Last enc state.
// Time variables
unsigned long previousMillis = 0; // For timing in loop.
/********************************************************************/

void setup() {
  // start serial port
  Serial.begin(9600);

  // Attach encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(EncOutputA), adjustSetpoint, CHANGE);

  // Start collection temps.
  sensors.begin();

  // Set the encoder pins.
  pinMode (EncOutputA, INPUT);
  digitalWrite(EncOutputA, HIGH);
  pinMode (EncOutputB, INPUT);
  digitalWrite(EncOutputB, HIGH);

  // Set the PWM pins.
  pinMode(PWM_Control, OUTPUT);

  // Set the pwmfreq to MOSFET to avoid noise.
  setPwmFrequency(10, 1024);

  // Initial state of encoder.
  aLastState = digitalRead(EncOutputA);

  // Start up the libraries
  display.begin();            // Initializes the display
  display.setBacklight(lcdBrightness);   // Set the brightness
  display.setColonOn(1);
}

void loop() {
  // Check temp and set pwm values according to intervall set in constants.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    //Serial.print(" Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperature readings
    //Serial.println("DONE");
    //Serial.print("Temperature is: ");
    temp = sensors.getTempCByIndex(0);
    //Serial.print(temp);
    int percentage = constrain(mapfloat(temp, setpoint, setpoint + diff, 0, 100), 0, 100);
    pwmValue = map(percentage, 0, 100, 0, 255);
    Serial.print(percentage);
    if (pwmValue < minFan)
    {
      analogWrite(PWM_Control, 0);
    } else
    {
      analogWrite(PWM_Control, pwmValue);
    }
  }
  int val = (setpoint * 100) + temp;
  display.print(val);
}

void adjustSetpoint()
{
  aState = digitalRead(EncOutputA);
  if (aState > aLastState) {
    if (digitalRead(EncOutputB) != aState) {
      setpoint = min(setpoint++, 99);
    } else {
      setpoint = max(setpoint--, 10);
    }
  }
  aLastState = aState;
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

