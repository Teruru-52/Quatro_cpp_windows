#ifndef ODOMETORY_HPP_
#define ODOMETORY_HPP_

#include "main.h"
#include <vector>
#include "hardware/encoder.h"
#include "hardware/imu.h"

namespace undercarriage
{
    class Odometory
    {
    private:
        hardware::Encoder encoder;
        hardware::IMU imu;

        float sampling_period; // [s]

    public:
        Odometory(float sampling_period);

        void Initialize();
        void Update();
        void ResetOdometory();
        std::vector<float> GetPosition();
        std::vector<float> GetVelocity();

        float v;
        float omega;
        float x;
        float y;
        float theta;
        std::vector<float> cur_pos;
        std::vector<float> cur_vel;
    };
} //  namespace undercarriage
#endif //  ODOMETORY_HPP_