#ifndef FUN_FORWARD_DYNAMICS_H_INCLUDED
#define FUN_FORWARD_DYNAMICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/aba.hpp>


namespace mecali
{
  casadi::Function get_forward_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
}

#endif // FUN_FORWARD_DYNAMICS_H_INCLUDED
