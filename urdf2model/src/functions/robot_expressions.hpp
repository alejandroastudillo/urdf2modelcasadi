#ifndef FUN_ROBOT_EXPRESSIONS_H_INCLUDED
#define FUN_ROBOT_EXPRESSIONS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>


namespace mecali
{
  casadi::Function get_robot_expressions(CasadiModel &cas_model, CasadiData &cas_data, std::vector<std::string> frame_names);
}

#endif // FUN_ROBOT_EXPRESSIONS_H_INCLUDED
