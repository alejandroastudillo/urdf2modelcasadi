#ifndef FUN_FORWARD_KINEMATICS_H_INCLUDED
#define FUN_FORWARD_KINEMATICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace mecali
{
  casadi::Function get_forward_kinematics_position(CasadiModel &cas_model, CasadiData &cas_data);
}

#endif // FUN_FORWARD_KINEMATICS_H_INCLUDED
