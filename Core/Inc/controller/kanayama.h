#ifndef UNDERCARRIAGE_KANAYAMA_HPP_
#define UNDERCARRIAGE_KANAYAMA_HPP_

#include "main.h"
#include <vector>
#include "trajectory.h"

namespace undercarriage
{
    class Kanayama
    {
    public:
        Kanayama(float Kx, float Ky, float Ktheta);

        void UpdateRef();
        std::vector<float> CalcInput(std::vector<float> cur_pos);

    private:
        trajectory::TurnLeft90 turnleft90;
        float Kx;
        float Ky;
        float Ktheta;
        std::vector<float> ref_pos;
        std::vector<float> ref_vel;
        float x_e;
        float y_e;
        float theta_e;
        std::vector<float> ref_u;
    };
} // namespace undercarriage

#endif //  UNDERCARRIAGE_KANAYAMA_HPP_