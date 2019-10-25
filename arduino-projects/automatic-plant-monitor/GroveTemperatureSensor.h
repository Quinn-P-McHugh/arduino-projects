/*
  GroveTemperatureSensor.h - Library for Grove Temperature Sensor v1.2
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#ifndef GroveTemperatureSensor_h
#define GroveTemperatureSensor_h

#include "Arduino.h"

class GroveTemperatureSensor
{
  public:
    GroveTemperatureSensor(int pin, int B = 4250, int R0 = 100000);
    double getTemperature();
    double convertToFahrenheit(double tempCelsius);
  private:
    int _pin;
    int _B;
    int _R0;
};

#endif

