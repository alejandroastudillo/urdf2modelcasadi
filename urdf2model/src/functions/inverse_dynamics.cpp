#include "inverse_dynamics.hpp"

casadi::Function get_inverse_dynamics(CasadiModel &cas_model, CasadiData &cas_data)
{
  // Set variables
  CasadiScalar q_sx = casadi::SX::sym("q", cas_model.nq);
  ConfigVectorCasadi q_casadi(cas_model.nq);
  pinocchio::casadi::copy(q_sx,q_casadi);

  CasadiScalar v_sx = casadi::SX::sym("v", cas_model.nv);
  TangentVectorCasadi v_casadi(cas_model.nv);
  pinocchio::casadi::copy(v_sx,v_casadi);

  CasadiScalar a_sx = casadi::SX::sym("a", cas_model.nv);
  TangentVectorCasadi a_casadi(cas_model.nv);
  pinocchio::casadi::copy(a_sx,a_casadi);

  // Call the Recursive Newton-Euler algorithm
  pinocchio::rnea(cas_model,cas_data,q_casadi,v_casadi,a_casadi);

  // Get the result from ABA into an SX
  casadi::SX tau_sx(cas_model.nv,1);
  pinocchio::casadi::copy( cas_data.tau, tau_sx );

  // Create the RNEA function
  casadi::Function rnea("rnea", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {tau_sx});

  return rnea;
}
