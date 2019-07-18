#ifndef FUN_INVERSE_DYNAMICS_H_INCLUDED
#define FUN_INVERSE_DYNAMICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/rnea.hpp>


namespace mecali
{
  casadi::Function get_inverse_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
}

#endif // FUN_INVERSE_DYNAMICS_H_INCLUDED
