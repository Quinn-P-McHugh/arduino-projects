/*
  GroveLightSensor.cpp - Library for Grove Light Sensor v1.1
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "GroveLightSensor.h"

GroveLightSensor::GroveLightSensor(int pinSig) {
  _pinSig = pinSig;
}

/*
 * Gets a reading from the light sensor and maps it to a specified range.
 */
int GroveLightSensor::read() {
  return map(analogRead(_pinSig), 0, 780, 0, 100);
}

