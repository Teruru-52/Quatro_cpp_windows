#ifndef HARDWARE_MOTOR_HPP_
#define HARDWARE_MOTOR_HPP_

#include "main.h"
#include "hardware/ir_sensor.h"

namespace hardware
{
    class Motor
    {
    private:
        // IRsensor battery;

        float bat_vol;
        int max_input;
        float duty_left;
        float duty_right;

    public:
        Motor(int max_input);

        void UpdateBatteryVoltage(float battery);
        int GetDuty(float input_vol);
        void Drive(float v_left, float v_right);
        void Brake();
        void Free();
    };
} // namespace hardware
#endif //  HARDWARE_MOTOR_HPP_