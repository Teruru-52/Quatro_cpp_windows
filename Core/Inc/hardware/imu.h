#ifndef HARDWARE_IMU_HPP_
#define HARDWARE_IMU_HPP_

#include "main.h"

namespace hardware
{
    class IMU
    {
    public:
        IMU(float sampling_period, float gyro_factor);

        uint8_t read_byte(uint8_t reg);
        void write_byte(uint8_t reg, uint8_t data);

        void Initialize();
        void CalcOffset();
        void Update();
        void UpdateGyro();
        // void UpdateAcc();
        float GetAngle();
        float GetAngularVelocity();

    private:
        float sampling_period;
        float gyro_factor;

        float theta;
        float gyro_z;
        float offset_gz;
    };
}
#endif // HARDWARE_IMU_HPP_