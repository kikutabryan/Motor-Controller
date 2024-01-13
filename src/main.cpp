#include <Arduino.h>
#include "PIDController.h"

#define RECEIVER_PIN 2
#define HALL_SENSOR_PIN A0

PIDController motorController(1.0, 0, 0);

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

void setup()
{
  motorController.setOutputLimits(1000, 2000);
  motorController.setIntegralLimits(1000, 2000);

  pinMode(RECEIVER_PIN, INPUT);
}

void loop()
{
  long receiverValue = pulseIn(RECEIVER_PIN, HIGH);
  int sensorValue = analogRead(HALL_SENSOR_PIN);

  double mappedReceiverValue = mapDouble(receiverValue, 1000, 2000, 1000, 2000);
  double mappedSensorValue = mapDouble(sensorValue, 0, 1023, 1000, 2000);

  motorController.compute(mappedReceiverValue, mappedSensorValue);
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}