l/*
  The following program uses Javadoc Documentation Style and the Arduino Style Guide:
  https://www.oracle.com/technetwork/java/javase/documentation/index-137868.html
  https://www.arduino.cc/en/Reference/APIStyleGuide

  Ball & Beam Project for Systems Dynamics and Controls - Fall 2018 & Spring 2019

  Reads a ball's position on a beam using a soft potentiometer and rotates
  a motor to tilt the beam and move the ball to a user-specified position
  along the beam.

  Created:  April 10th, 2019
  Authors:  Team "Hashtag Tilted"
            - Peter Genovese
            - Scott Hood
            - Leslie Maier
            - Quinn McHugh
*/

#include "AccelStepper.h"

/* Initialize variables stepper motor */
const int DIR_PIN = 5;          // The direction pin connected to "DIR" on the stepper motor driver
const int STEP_PIN = 4;         // The step pin connected to "STEP" on the stepper motor driver
const int STEPPING_MODE = 1;    // The motor's stepping mode -- 1 for full stepping, 2 for half stepping, 4 for quarter stepping, 8 for 1/8th stepping, etc.
const int STEPS_PER_REV = 200;  // The steps/revolution of the motor.
const int PULLEY_RATIO = 3;     // The torque ratio of the pulleys that connect the motor shaft and the beam shaft.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* Spcify soft & hard potentiometer pins */
const int SOFT_POT_PIN = A0;    // The analog pin connected to the soft potentiometer
const int HARD_POT_PIN = A1;    // The analog pin connected to the hard potentiometer

/* PID controller variables */
double K_p = 2 ;        // Proportional gain constant
double K_d = 1000;      // Derivative gain constant
double K_i = 3;         // Integral gain constant

double ballPos;         // The current position of the ball.
double desiredPos = 0;  // The desired position of the ball.

unsigned long currentTime = 0;   // Tells the time the current computation started.
unsigned long startTime = micros();     // Tells the time the last computation ended.
const unsigned long SAMPLE_RATE = 0.1 * pow(10,6);  // [microseconds/sample] The amount of time that should pass before the PID algorithm calculates a new U value.

double err = 0;         // Difference between current ball position and previous ball position.
double errDir = 0;      // The rate at which the error is changing.
double errInt = 0;      // The integral of the error.
double errIntOld = 0;   // Holds previous sum of area under curve for calculations.
double errOld = 0;      // Holds previous error value for calculations.
double U = 0;           // Value used to determine motor speed and direction?
double Usum = 0;        // running tally of current step / limits angle
double UStep = 0;       // Steps to move the motor.

void setup()
{
  /* Initialize GPIO pins */
  pinMode(SOFT_POT_PIN, INPUT);
  pinMode(HARD_POT_PIN, INPUT);

  stepper.setMaxSpeed(2000);      // [steps/s]
  stepper.setSpeed(1000);         // [steps/s]
  stepper.setAcceleration(1000);  // [steps/s]

  Serial.begin(250000);
}

void loop() {
  currentTime = micros() / 10000;
  if (currentTime - startTime >= SAMPLE_RATE) {
    /* Determine the ball and desired positions from the soft and hard potentiometers */
    double ballPos = mapDouble(analogRead(SOFT_POT_PIN), 0, 1009, 0, 20);       // Soft potentiometer reading mapped to inches
    double desiredPos = mapDouble(analogRead(HARD_POT_PIN), 0, 1016.5, 0, 20);  // Hard potentiometer reading mapped to inches

    /* Print values to  serial monitor for troubleshooting */
    Serial.print("Desired Pos: ");
    Serial.print(desiredPos, 4);
    Serial.print("  |  Ball Pos: ");
    Serial.print(ballPos, 5);
    Serial.println();

    /* The math from Zhang's class */
    err = -desiredPos + ballPos;
    Serial.print("err ");
    Serial.print(err, 5);
    Serial.println();
    errDir = (err - errOld) / (currentTime - startTime);
    Serial.print("errDir ");
    Serial.print(errDir, 5);
    Serial.println();
    errInt = errIntOld + (err * (currentTime - startTime));
    Serial.print("errInt ");
    Serial.print(errInt, 5);
    Serial.println();

    /* Record values for the next run */
    errOld = err;
    errIntOld = errInt;

    U = K_p * err + K_d * errDir + K_i * errInt;  // U is used to drive the motor.
    Serial.print("U: ");
    Serial.print(U, 5);
    Serial.println();
    Serial.print("UStep: ");
    Serial.print(UStep, 5);
    Serial.println();

    UStep = (long) -U / 150;   // Scaling factor from U to Steps for the motor

    /* Check that the motor does not tilt beyond a set limit (30 steps = 18 degrees) */
    long tiltUpperLimit_steps = degrees2Steps(30);
    long tiltLowerLimit_steps = degrees2Steps(-30);
    if (UStep >= tiltUpperLimit_steps) {
      UStep = tiltUpperLimit_steps;
    }
    else if (UStep <= tiltLowerLimit_steps) {
      UStep = tiltLowerLimit_steps;
    }

    Serial.print("U value: ");
    Serial.print(U, 8);     // This is so I can troubleshoot this value when things start flinging
    Serial.print("  |   Usum value: ");
    Serial.print(Usum, 8);  // This is so I can troubleshoot this value when things start flinging
    Serial.println();

    startTime = currentTime;  // Reset the the start time to determine when new values should be calculated should be run
  }

  stepper.moveTo(UStep * STEPPING_MODE);  // Set the motor's target position, regardless of whether or not a new U value was calculated.
  stepper.run();  // Determine if the motor needs to make a step and, if so, step the motor single step.
}

/* Maps a number on one number range, [fromLow, fromHigh], to another range, [toLow, toHigh].
 *
 * @param value     The value to be mapped.
 * @param fromLow   The lower bound of the original range.
 * @param fromHigh  The upper bound of the original range.
 * @param toLow     The lower bound of the final range.
 * @param toHigh    The upper bound of the final range.
 * @return  The value mapped onto the final range.
 */
double mapDouble(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/*
 * Converts beam degrees to motor steps.
 *
 * @param deg The number of degrees to be converted to steps.
 */
long degrees2Steps(double deg) {
  return deg * (1/360) * STEPS_PER_REV * PULLEY_RATIO * STEPPING_MODE;    // [deg] * [1rev/360deg] * [steps/rev] * (pulley ratio) * (stepping mode scaling factor)
}
