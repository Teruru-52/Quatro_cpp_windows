#include "trajectory.h"

namespace trajectory
{
    TurnLeft90::TurnLeft90()
        : index(0) {}

    int TurnLeft90::GetTrajectoryIndex()
    {
        return index;
    }

    void TurnLeft90::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void TurnLeft90::UpdateRef()
    {
        ref_pos = ref_pos_csv[index];
        // ref_vel = ref_vel_csv[index];
        index++;
    }

    std::vector<float> TurnLeft90::GetRefPosition()
    {
        return ref_pos;
    }

    std::vector<float> TurnLeft90::GetRefVelocity()
    {
        return ref_vel;
    }

    PivotTurn::PivotTurn()
        : index(0) {}

    int PivotTurn::GetTrajectoryIndex()
    {
        return index;
    }

    void PivotTurn::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void PivotTurn::UpdateRef()
    {
        ref_w = ref_w_csv[index];
        index++;
    }

    float PivotTurn::GetRefVelocity()
    {
        return ref_w;
    }
}