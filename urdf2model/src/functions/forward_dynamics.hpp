#ifndef FUN_FORWARD_DYNAMICS_H_INCLUDED
#define FUN_FORWARD_DYNAMICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>


namespace mecali
{
  casadi::Function get_forward_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_mass_inverse(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_forward_dynamics_derivatives(CasadiModel &cas_model, CasadiData &cas_data, std::string type );
}

#endif // FUN_FORWARD_DYNAMICS_H_INCLUDED
