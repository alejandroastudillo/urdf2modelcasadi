#include "inverse_dynamics.hpp"

namespace mecali
{

  casadi::Function get_inverse_dynamics(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi( cas_model.nv );
    pinocchio::casadi::copy( v_sx, v_casadi );

    CasadiScalar        a_sx = casadi::SX::sym("a", cas_model.nv);
    TangentVectorCasadi a_casadi(cas_model.nv);
    pinocchio::casadi::copy( a_sx, a_casadi );

    // Call the Recursive Newton-Euler algorithm
    pinocchio::rnea( cas_model, cas_data, q_casadi, v_casadi, a_casadi );

    // Get the result from ABA into an SX
    casadi::SX          tau_sx( cas_model.nv, 1);
    pinocchio::casadi::copy( cas_data.tau, tau_sx );

    // Create the RNEA function
    casadi::Function    rnea("rnea", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {tau_sx});

    return rnea;
  }

  casadi::Function get_generalized_gravity(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi( cas_model.nv );
    pinocchio::casadi::copy( v_sx, v_casadi );

    CasadiScalar        a_sx = casadi::SX::sym("a", cas_model.nv);
    TangentVectorCasadi a_casadi(cas_model.nv);
    pinocchio::casadi::copy( a_sx, a_casadi );

    // Call the Recursive Newton-Euler algorithm
    pinocchio::rnea( cas_model, cas_data, q_casadi, v_casadi, a_casadi );
    pinocchio::computeGeneralizedGravity(cas_model, cas_data, q_casadi);

    // Get the result from ABA into an SX
    casadi::SX          g_sx( cas_model.nv, 1);
    pinocchio::casadi::copy( cas_data.g, g_sx );

    // Create the RNEA function
    casadi::Function    generalized_gravity("generalized_gravity", casadi::SXVector {q_sx}, casadi::SXVector {g_sx});

    return generalized_gravity;
  }

  casadi::Function get_coriolis(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi( cas_model.nv );
    pinocchio::casadi::copy( v_sx, v_casadi );

    CasadiScalar        a_sx = casadi::SX::sym("a", cas_model.nv);
    TangentVectorCasadi a_casadi(cas_model.nv);
    pinocchio::casadi::copy( a_sx, a_casadi );

    // Call the Recursive Newton-Euler algorithm
    pinocchio::rnea( cas_model, cas_data, q_casadi, v_casadi, a_casadi );
    pinocchio::computeCoriolisMatrix(cas_model, cas_data, q_casadi, v_casadi);

    // Get the result from ABA into an SX
    casadi::SX          C_sx( cas_model.nv, cas_model.nv);
    pinocchio::casadi::copy( cas_data.C, C_sx );

    // Create the RNEA function
    casadi::Function    coriolis("coriolis", casadi::SXVector {q_sx, v_sx}, casadi::SXVector {C_sx});

    return coriolis;
  }
  casadi::Function get_joint_torque_regressor(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi( cas_model.nv );
    pinocchio::casadi::copy( v_sx, v_casadi );

    CasadiScalar        a_sx = casadi::SX::sym("a", cas_model.nv);
    TangentVectorCasadi a_casadi(cas_model.nv);
    pinocchio::casadi::copy( a_sx, a_casadi );

    // Call the Recursive Newton-Euler algorithm
    pinocchio::rnea( cas_model, cas_data, q_casadi, v_casadi, a_casadi );
    pinocchio::computeJointTorqueRegressor(cas_model, cas_data, q_casadi, v_casadi, a_casadi);

    // Get the result from ABA into an SX
    casadi::SX          JTR_sx( cas_model.nv, 10*(cas_model.nv));
    pinocchio::casadi::copy( cas_data.jointTorqueRegressor, JTR_sx );

    // Create the RNEA function
    casadi::Function    joint_torque_regressor("joint_torque_reg", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {JTR_sx});

    return joint_torque_regressor;
  }

}
