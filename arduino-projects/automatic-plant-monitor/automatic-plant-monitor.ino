/*
  The following file uses Javadoc Documentation Style and the Arduino Style Guide:
  https://www.oracle.com/technetwork/java/javase/documentation/index-137868.html
  https://www.arduino.cc/en/Reference/APIStyleGuide

  Automatic Plant Monitor - Spring 2019

  Monitors environmental conditions around a plant using Grove sensors
  and outputs data to LCD screen

  Created:  March 9th, 2019
  Author:  Quinn McHugh
  
  Changelog:
    Modified: (Date)
    Author:   (Name)
    Change:   (Description of changes)
*/

#include "rgb_lcd.h"
#include "GroveLightSensor.h"
#include "GroveTemperatureSensor.h"
#include "GroveButton.h"
#include "SparkFunSoilMoistureSensor.h"

rgb_lcd lcd;
GroveLightSensor lightSensor(A0);
GroveTemperatureSensor tempSensor(A1);
GroveButton lcdButton(A2);
SparkFunSoilMoistureSensor moistureSensor(A3, 7);

enum displayState {
  TEMPERATURE,    // Display temperature
  LIGHT,          // Display light level
  SOIL_MOISTURE,  // Display soil moisture level
};

const int motorPin = 6;

displayState lcdDisplayState = (displayState) 0; // Initial display on LCD
double sensorSampleRate = 0.5;   // [seconds/sample]
double motorSpeed = 100;
int motorState = LOW;

unsigned long previousMillisSensor = 0;
unsigned long previousMillisMotor = 0;

void setup() {
  lcd.begin(16,2);
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  double tempFahrenheit = tempSensor.convertToFahrenheit(tempSensor.getTemperature());
  double lightIntensity = lightSensor.read();
  double soilMoistureValue = moistureSensor.read();

  checkLCDDisplayState();

  // Move motor when soil moisture sensor goes below a certain threshold.
  if (soilMoistureValue < 775) {
    if (currentMillis - previousMillisMotor > motorSpeed) {
      previousMillisMotor = currentMillis;
      moveMotor();
    }
  }

  // Output new sensor value to LCD at a specified rate.
  if (currentMillis - previousMillisSensor >=  sensorSampleRate * pow(10, 3)) {
    previousMillisSensor = currentMillis;
    
    lcd.clear();
    switch(lcdDisplayState) {
      case TEMPERATURE:
        lcd.print("Outside Temp: ");
        lcd.setCursor(0, 1);
        lcd.print(String(tempFahrenheit) + " [" + (char) 223 + "F]");
        break;
      case LIGHT:
        lcd.print("Light Intensity: ");
        lcd.setCursor(0, 1);
        lcd.print(String(lightIntensity) + " [lumens]");
        break;
      case SOIL_MOISTURE:
        lcd.print("Soil Moisture");
        lcd.setCursor(0, 1);
        lcd.print(String(soilMoistureValue));
        break;
    }
  }
}

/*
 * Checks if the LCD button is pressed and, if so, cycles through LCD display states.
 */
void checkLCDDisplayState() {
  if (lcdButton.read() == HIGH) {
    if (lcdDisplayState >= 2) {
      lcdDisplayState = (displayState) 0;
    }
    else {
      lcdDisplayState = (displayState) lcdDisplayState + 1;
    }
  }
}

/*
 * Moves the motor one step.
 */
void moveMotor() {
  if (motorState == LOW) {
    motorState = HIGH;
  }
  else {
    motorState = LOW;
  }
  digitalWrite(motorPin, motorState);
}

