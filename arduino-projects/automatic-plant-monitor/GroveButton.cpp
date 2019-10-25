/*
  GroveButton.cpp - Library for Grove Button Sensor v1.2
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "GroveButton.h"

GroveButton::GroveButton(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT);
}

int GroveButton::read() {
  return digitalRead(_pin);
}

