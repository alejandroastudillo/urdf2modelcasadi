#ifndef FUN_INVERSE_DYNAMICS_H_INCLUDED
#define FUN_INVERSE_DYNAMICS_H_INCLUDED

#include <casadi/casadi.hpp>
#include "pinocchio/math/casadi.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"


// Typedef
  typedef casadi::SX                          CasadiScalar;

  typedef pinocchio::ModelTpl<CasadiScalar>   CasadiModel;
  typedef CasadiModel::Data                   CasadiData;

  typedef CasadiModel::ConfigVectorType       ConfigVectorCasadi;
  typedef CasadiModel::TangentVectorType      TangentVectorCasadi;

namespace mecali
{
  casadi::Function get_inverse_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
}

#endif // FUN_INVERSE_DYNAMICS_H_INCLUDED
