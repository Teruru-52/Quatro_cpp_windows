#include "odometory.h"

#include <cmath>

namespace undercarriage
{
  Odometory::Odometory(float sampling_period)
      : encoder(sampling_period, 0.0125),
        imu(sampling_period, 16.4),
        sampling_period(sampling_period),
        x(0),
        y(0) {}

  void Odometory::Initialize()
  {
    imu.Initialize();
    imu.CalcOffset();
  }

  void Odometory::ResetOdometory()
  {
    x = 0;
    y = 0;
  }

  void Odometory::Update()
  {
    encoder.Update();
    imu.Update();

    v = encoder.GetVelocity();
    omega = imu.GetAngularVelocity();
    theta = imu.GetAngle();
    x += v * cos(theta) * sampling_period;
    y += v * sin(theta) * sampling_period;
  }

  std::vector<float> Odometory::GetPosition()
  {
    cur_pos[0] = x;
    cur_pos[1] = y;
    cur_pos[2] = theta;
    return cur_pos;
  }

  std::vector<float> Odometory::GetVelocity()
  {
    cur_vel[0] = v;
    cur_vel[1] = omega;
    return cur_vel;
  }
} //  namespace undercarriage