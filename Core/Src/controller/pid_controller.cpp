#include "controller/pid_controller.h"

Integrator::Integrator(float control_period)
    : control_period(control_period),
      error_sum(0) {}

float Integrator::Update(float error)
{
    error_sum += error * control_period;
    return error_sum;
}

float Integrator::ResetIntegrator()
{
    error_sum = 0.0;
}

Differentiator::Differentiator(float tf, float control_period)
    : tf(tf),
      control_period(control_period),
      coeff(tf / (tf + control_period)),
      pre_error(0),
      pre_deriv(0) {}

float Differentiator::Update(float error)
{
    float deriv = (error - pre_error) / control_period;
    pre_deriv = coeff * pre_deriv + (1.0f - coeff) * deriv;
    pre_error = error;
    return pre_deriv;
}

float Differentiator::ResetDifferentiator()
{
    pre_error = 0.0;
}

PID::PID(float kp, float ki, float kd, float tf, float control_period)
    : kp(kp),
      ki(ki),
      kd(kd),
      integrator(control_period),
      differentiator(tf, control_period) {}

float PID::Update(float error)
{
    return kp * error + ki * integrator.Update(error) + kd * differentiator.Update(error);
}

float PID::ResetPID()
{
    integrator.ResetIntegrator();
    differentiator.ResetDifferentiator();
}