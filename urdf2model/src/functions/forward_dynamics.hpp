#include <casadi/casadi.hpp>
#include "pinocchio/math/casadi.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

// Typedef
  typedef casadi::SX                          CasadiScalar;

  typedef pinocchio::ModelTpl<CasadiScalar>   CasadiModel;
  typedef CasadiModel::Data                   CasadiData;

  typedef CasadiModel::ConfigVectorType       ConfigVectorCasadi;
  typedef CasadiModel::TangentVectorType      TangentVectorCasadi;

casadi::Function get_forward_dynamics(CasadiModel &cas_model, CasadiData &cas_data);
