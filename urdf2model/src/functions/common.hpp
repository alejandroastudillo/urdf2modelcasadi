#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include <casadi/casadi.hpp>
#include <pinocchio/math/casadi.hpp>
#include <pinocchio/multibody/model.hpp>

namespace mecali
{
  typedef casadi::Dict    Dictionary;

  typedef double                              Scalar;
  typedef casadi::SX                          CasadiScalar;

  typedef pinocchio::ModelTpl<Scalar>         Model;
  typedef Model::Data                         Data;

  typedef pinocchio::ModelTpl<CasadiScalar>   CasadiModel;
  typedef CasadiModel::Data                   CasadiData;

  typedef Model::ConfigVectorType             ConfigVector;
  typedef Model::TangentVectorType            TangentVector;

  typedef CasadiModel::ConfigVectorType       ConfigVectorCasadi;
  typedef CasadiModel::TangentVectorType      TangentVectorCasadi;
}
#endif // COMMON_H_INCLUDED
