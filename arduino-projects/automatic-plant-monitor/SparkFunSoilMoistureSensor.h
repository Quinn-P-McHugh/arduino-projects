/*
  GroveLightSensor.h - Library for SparkFun Moisture Sensor
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#ifndef SparkFunSoilMoistureSensor_h
#define SparkFunSoilMoistureSensor_h

#include "Arduino.h"

class SparkFunSoilMoistureSensor
{
  public:
    SparkFunSoilMoistureSensor(int pinSig, int pinVCC);
    int read();
  private:
    int _pinSig;
    int _pinVCC;
};

#endif

