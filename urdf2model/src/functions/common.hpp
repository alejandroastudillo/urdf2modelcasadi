#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include <casadi/casadi.hpp>
#include <pinocchio/math/casadi.hpp>
#include <pinocchio/multibody/model.hpp>

namespace mecali
{
  typedef casadi::Dict    Dictionary;

  typedef casadi::SX                          CasadiScalar;

  typedef pinocchio::ModelTpl<CasadiScalar>   CasadiModel;
  typedef CasadiModel::Data                   CasadiData;

  typedef CasadiModel::ConfigVectorType       ConfigVectorCasadi;
  typedef CasadiModel::TangentVectorType      TangentVectorCasadi;
}
#endif // COMMON_H_INCLUDED
