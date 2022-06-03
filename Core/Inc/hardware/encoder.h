#ifndef HARDWARE_ENCODER_HPP_
#define HARDWARE_ENCODER_HPP_

#include "main.h"

namespace hardware
{
    class Encoder
    {
    public:
        Encoder(float sampling_period, float tire_radius);

        void Update();
        void Update_L();
        void Update_R();
        float GetAngularVelocity(int16_t pulse);
        float GetVelocity();

    private:
        float gear_ratio = 11.0f / 43.0f;
        float sampling_period; // [ms]
        float tire_radius;     // [m]
        const int16_t ppr = 4096;
        int16_t pulse_left;
        int16_t pulse_right;
    };
} // namespace hardware

#endif // HARDWARE_ENCODER_HPP_