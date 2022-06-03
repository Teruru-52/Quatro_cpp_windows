#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "main.h"
#include <vector>
#include "hardware/motor.h"
#include "controller/pid_controller.h"
#include "controller/kanayama.h"

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(float control_period);

        void UpdateBatteryVoltage(float bat_vol);
        void PartyTrick(std::vector<float> cur_pos, std::vector<float> cur_vel);
        void PivotTurn180(std::vector<float> cur_vel);
        void KanayamaUpdateRef();
        void KanayamaTurnLeft90(std::vector<float> cur_pos, std::vector<float> cur_vel);
        void GoStraight(std::vector<float> cur_pos, std::vector<float> cur_vel, std::vector<uint32_t> ir_data);
        void InputVelocity(float input_v, float input_w);
        void MotorTest();

    private:
        hardware::Motor motor;
        PID pid_angle;
        PID pid_rotational_vel;
        PID pid_traslational_vel;
        PID pid_ir_sensor_left;
        PID pid_ir_sensor_right;
        undercarriage::Kanayama kanayama;
        trajectory::PivotTurn pivot_turn;

        float v_left;
        float v_right;
        float u_w;
        float u_v;
        // std::vector<float> ref_pos;
        std::vector<float> ref_vel;
        float ref_w;
        const float Tp1 = 37.67;
        const float Kp = 105.1;
        const float v_straight = 0.936196798079597;
        const float ir_straight = 1000;
    };
} // namespace undercarriage

#endif //  CONTROLLER_HPP_