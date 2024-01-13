#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
public:
    PIDController(double Kp, double Ki, double Kd);
    double compute(double setpoint, double input);
    void setOutputLimits(double min, double max);
    void setIntegralLimits(double min, double max);

private:
    double _Kp, _Ki, _Kd;
    double _minOutput, _maxOutput;
    double _minIntegral, _maxIntegral;
    double _integral, _previousInput, _previousTime;
};

#endif