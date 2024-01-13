#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(double Kp, double Ki, double Kd)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _integral(0), _previousInput(0), _previousTime(millis()), _minOutput(1000), _maxOutput(2000), _minIntegral(1000), _maxIntegral(2000) {}

double PIDController::compute(double setpoint, double input)
{
    double now = millis();
    double timeChange = (now - _previousTime) / 1000.0; // Time change in seconds

    double error = setpoint - input;
    _integral += error * timeChange;
    if (_integral > _maxIntegral)
        _integral = _maxOutput;
    else if (_integral < _minIntegral)
        _integral = _minOutput;

    double derivative = (_previousInput - input) / timeChange;
    _previousInput = input;
    _previousTime = now;

    double output = _Kp * error + _Ki * _integral + _Kd * derivative;
    if (output > _maxOutput)
        output = _maxOutput;
    else if (output < _minOutput)
        output = _minOutput;

    return output;
}

void PIDController::setOutputLimits(double min, double max)
{
    _minOutput = min;
    _maxOutput = max;
}

void PIDController::setIntegralLimits(double min, double max)
{
    _minIntegral = min;
    _maxIntegral = max;
}