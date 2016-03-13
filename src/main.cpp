#include <Arduino.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

/*
 * This example uses the ZumoMotors library to follow a black line.
 */

#define LED_PIN                 13
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
ZumoReflectanceSensorArray reflectanceSensors;
unsigned int sensorValues[NUM_SENSORS];
double Kp = 0;
int offset = 0;
int integral = 0;
Pushbutton button(ZUMO_BUTTON);
ZumoMotors motors;

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 400;

void setup()
{
  delay(500);

  // Initialize the reflectance sensors module
  reflectanceSensors.init();

  pinMode(LED_PIN, OUTPUT);
  // turn on Arduino's LED to indicate we are in calibration mode
  digitalWrite(LED_PIN, HIGH);
  int i;
  for (i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
    reflectanceSensors.calibrate();

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }

  motors.setSpeeds(0,0);
  // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(LED_PIN, LOW);

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  /*
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println(offset);
  */
  // Wait for the user button to be pressed and released
  button.waitForButton();

  // Read the offset value after setting zumo in the right position.
  offset = reflectanceSensors.readLine(sensorValues);
  Kp = offset / MAX_SPEED;

  Serial.print(offset);
  Serial.print(' ');
  Serial.println(Kp);

  delay(1000);
}

// read calibrated sensor values and obtain a measure of the line position from 0 to 5000
void loop()
{
  digitalWrite(LED_PIN, HIGH);
  int position = reflectanceSensors.readLine(sensorValues);
  int error = position - offset;
  integral += error;
  if ((integral < 0) == (error < 0)) {
    integral = 0;
  }
  int turn = Kp * error + integral;
  int leftSpeed = MAX_SPEED + turn;
  int rightSpeed = MAX_SPEED - turn;
  /*
  Serial.print(position);
  Serial.print(' ');
  Serial.print(leftSpeed);
  Serial.print(' ');
  Serial.println(rightSpeed);
  */

  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  if (leftSpeed < 0)
    leftSpeed = 0;
  if (rightSpeed < 0)
    rightSpeed = 0;
  if (leftSpeed > MAX_SPEED)
    leftSpeed = MAX_SPEED;
  if (rightSpeed > MAX_SPEED)
    rightSpeed = MAX_SPEED;

  // Set the motor speeds
  motors.setSpeeds(leftSpeed, rightSpeed);
}
