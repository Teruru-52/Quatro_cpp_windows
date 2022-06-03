#include "controller/controller.h"

namespace undercarriage
{
    Controller::Controller(float control_period)
        : motor(1000),
          pid_angle(2.0, 2.0, 0.05, 0.0, control_period),
          pid_rotational_vel(1.59, 60.0, -0.00213, 0.0, control_period),
          pid_traslational_vel(19.72, 52.0, 1.548, 0.0, control_period),
          pid_ir_sensor_left(1.0, 0.0, 0.0, 0.0, control_period),
          pid_ir_sensor_right(1.0, 0.0, 0.0, 0.0, control_period),
          kanayama(1.0, 1.0, 1.0) {}

    void Controller::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Controller::PartyTrick(std::vector<float> cur_pos, std::vector<float> cur_vel)
    {
        u_v = pid_traslational_vel.Update(-cur_vel[0]);
        u_w = pid_angle.Update(-cur_pos[2]) + pid_rotational_vel.Update(-cur_vel[1]);
        InputVelocity(u_v, u_w);
    }

    void Controller::PivotTurn180(std::vector<float> cur_vel)
    {
        pivot_turn.UpdateRef();
        ref_w = pivot_turn.GetRefVelocity();
        u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1 * ref_w / Kp;
        InputVelocity(0.0, u_w);
    }

    void Controller::KanayamaUpdateRef()
    {
        kanayama.UpdateRef();
    }

    void Controller::KanayamaTurnLeft90(std::vector<float> cur_pos, std::vector<float> cur_vel)
    {
        ref_vel = kanayama.CalcInput(cur_pos);
        u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]);
        u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1 * ref_w / Kp;
        InputVelocity(u_v, u_w);
    }

    void Controller::GoStraight(std::vector<float> cur_pos, std::vector<float> cur_vel, std::vector<uint32_t> ir_data)
    {
        u_v = pid_traslational_vel.Update(v_straight - cur_vel[0]);
        u_w = pid_ir_sensor_left.Update((float)(ir_straight - ir_data[2])) + pid_ir_sensor_right.Update((float)(ir_straight - ir_data[3])) + pid_angle.Update(-cur_pos[0]);
        InputVelocity(u_v, u_w);
    }

    void Controller::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    void Controller::MotorTest()
    {
        motor.Drive(1.5, 1.5); // voltage [V]
    }
}