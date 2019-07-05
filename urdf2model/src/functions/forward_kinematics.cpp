#include "forward_kinematics.hpp"

casadi::Function get_forward_kinematics_position(CasadiModel &cas_model, CasadiData &cas_data)
{
  int                 EE_idx = cas_model.nframes-1;
  // Set variables
  CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
  ConfigVectorCasadi  q_casadi(cas_model.nq);
  pinocchio::casadi::copy( q_sx, q_casadi );

  // Call the forward kinematics function
  pinocchio::forwardKinematics(     cas_model,   cas_data,    q_casadi);
  pinocchio::updateFramePlacements( cas_model,   cas_data);

  // Get the result from FK
  CasadiScalar        pos_sx(3,1);
  pinocchio::casadi::copy( cas_data.oMf[EE_idx].translation(), pos_sx );

  // Create the Forward Kinematics function
  casadi::Function    fk( "fk", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx} );

  return fk;
}
