/*
  GroveButton.h - Library for Grove Button v1.1
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#ifndef GroveButton_h
#define GroveButton_h

#include "Arduino.h"

class GroveButton
{
  public:
    GroveButton(int pin);
    int read();
  private:
    int _pin;
};

#endif

