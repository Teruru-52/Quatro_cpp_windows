#include "controller/kanayama.h"

namespace undercarriage
{
    Kanayama::Kanayama(float Kx, float Ky, float Ktheta)
        : Kx(Kx),
          Ky(Ky),
          Ktheta(Ktheta),
          ref_pos({0.0, 0.0, 0.0}),
          ref_vel({0.0, 0.0}),
          ref_u({0.0, 0.0}) {}

    void Kanayama::UpdateRef()
    {
        turnleft90.UpdateRef();
        ref_pos = turnleft90.GetRefPosition();
        ref_vel = turnleft90.GetRefVelocity();
    }

    std::vector<float> Kanayama::CalcInput(std::vector<float> cur_pos)
    {
        x_e = (ref_pos[0] - cur_pos[0]) * cos(cur_pos[2]) + (ref_pos[1] - cur_pos[1]) * sin(cur_pos[2]);
        y_e = -(ref_pos[0] - cur_pos[0]) * sin(cur_pos[2]) + (ref_pos[1] - cur_pos[1]) * cos(cur_pos[2]);
        theta_e = ref_pos[2] - cur_pos[2];

        ref_u[0] = ref_vel[0] * cos(theta_e) + Kx * x_e;
        ref_u[1] = ref_vel[1] + ref_vel[0] * (Ky * y_e + Ktheta * sin(theta_e));

        return ref_u;
    }
}