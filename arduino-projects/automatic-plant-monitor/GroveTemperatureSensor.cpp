/*
  GroveTemperatureSensor.cpp - Library for Grove Temperature Sensor v1.2
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "GroveTemperatureSensor.h"
#include <math.h>

GroveTemperatureSensor::GroveTemperatureSensor(int pin, int B, int R0) {
  _pin = pin;
  _B = B;
  _R0 = R0;
}

/*
 * Gets the temperature of the surrounding environment.
 *  
 * @return The temperature in degrees Celsius.
 */
double GroveTemperatureSensor::getTemperature() {
  int a = analogRead(_pin);
  double R = 1023.0/a-1.0;
  R = _R0*R;
  double temperature = 1.0/(log(R/_R0)/_B+1/298.15)-273.15; // Convert to temperature via datasheet
  return temperature;
} 

/*
 * Converts the a temperature in degrees Celsius to degrees Fahrenheit.
 *
 * @param tempCelsius The temperature in degrees Celsius
 * @return The temperature in degrees Fahrenheit.
 */
double GroveTemperatureSensor::convertToFahrenheit(double tempCelsius) {
  return tempCelsius * 1.8 + 32;
}

