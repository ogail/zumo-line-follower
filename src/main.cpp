#include <Arduino.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>

/*
 * This example uses the ZumoMotors library to follow a black line.
 */

#define LED_PIN                 13
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

ZumoMotors motors;

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(LED_PIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  digitalWrite(LED_PIN, HIGH);

  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  unsigned int position = qtra.readLine(sensorValues);
  Serial.println(position);
  if (position > 500)
  {
    motors.setSpeeds(200, 200);
  }
  else
  {
    motors.setSpeeds(50, 50);
  }

  delay(250);
}
