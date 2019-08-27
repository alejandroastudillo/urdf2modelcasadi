#ifndef FUN_FORWARD_KINEMATICS_H_INCLUDED
#define FUN_FORWARD_KINEMATICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace mecali
{
  casadi::Function get_forward_kinematics(CasadiModel &cas_model, CasadiData &cas_data, std::string content, std::vector<std::string> frame_names);
}

#endif // FUN_FORWARD_KINEMATICS_H_INCLUDED
