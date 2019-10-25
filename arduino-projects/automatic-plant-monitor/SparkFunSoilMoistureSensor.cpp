/*
  GroveLightSensor.cpp - Library for Grove Light Sensor v1.1
  Created by Quinn McHugh, March 9th, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "SparkFunSoilMoistureSensor.h"

SparkFunSoilMoistureSensor::SparkFunSoilMoistureSensor(int pinSig, int pinVCC) {
  _pinSig = pinSig;
  _pinVCC = pinVCC;
  pinMode(_pinVCC, OUTPUT);
  digitalWrite(_pinVCC, LOW);
}

/*
 * Gets a reading from the soil moisture sensor.
 */
int SparkFunSoilMoistureSensor::read() {
  digitalWrite(_pinVCC, HIGH);
  delay(10);
  int reading = analogRead(_pinSig);
  digitalWrite(_pinSig, LOW);
  return reading;
}

