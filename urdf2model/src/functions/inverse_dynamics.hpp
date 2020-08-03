#ifndef FUN_INVERSE_DYNAMICS_H_INCLUDED
#define FUN_INVERSE_DYNAMICS_H_INCLUDED

#include "common.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include "pinocchio/algorithm/regressor.hpp"


namespace mecali
{
  casadi::Function get_inverse_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_generalized_gravity(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_coriolis(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_joint_torque_regressor(CasadiModel &cas_model, CasadiData &cas_data);
  casadi::Function get_generalized_gravity_derivatives(CasadiModel &cas_model, CasadiData &cas_data);
}

#endif // FUN_INVERSE_DYNAMICS_H_INCLUDED
