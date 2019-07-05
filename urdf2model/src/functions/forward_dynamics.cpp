#include "forward_dynamics.hpp"


casadi::Function get_forward_dynamics(CasadiModel &cas_model, CasadiData &cas_data)
{
  // Set variables
  CasadiScalar q_sx = casadi::SX::sym("q", cas_model.nq);
  ConfigVectorCasadi q_casadi(cas_model.nq);
  pinocchio::casadi::copy(q_sx,q_casadi); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

  CasadiScalar v_sx = casadi::SX::sym("v", cas_model.nv);
  TangentVectorCasadi v_casadi(cas_model.nv);
  pinocchio::casadi::copy(v_sx,v_casadi); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

  CasadiScalar tau_sx = casadi::SX::sym("tau", cas_model.nv);
  TangentVectorCasadi tau_casadi(cas_model.nv);
  pinocchio::casadi::copy(tau_sx,tau_casadi); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

  // Call the Articulated-body algorithm
  pinocchio::aba(cas_model,cas_data,q_casadi,v_casadi,tau_casadi);

  // Get the result from ABA into an SX
  CasadiScalar ddq_sx(cas_model.nv,1);
  pinocchio::casadi::copy( cas_data.ddq, ddq_sx );

  // Create the ABA function
  casadi::Function aba("aba", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_sx});

  return aba;
}
