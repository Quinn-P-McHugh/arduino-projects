/*
  GroveLightSensor.h - Library for Grove Light Sensor v1.1
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#ifndef GroveLightSensor_h
#define GroveLightSensor_h

#include "Arduino.h"

class GroveLightSensor
{
  public:
    GroveLightSensor(int pinSig);
    int read();
  private:
    int _pinSig;
};

#endif

