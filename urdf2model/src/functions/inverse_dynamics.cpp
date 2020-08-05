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

  casadi::Function get_generalized_gravity_derivatives(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    // Output variable
    CasadiScalar          g_partial_dq_sx( cas_model.nv, cas_model.nv);
    CasadiData::MatrixXs  g_partial_dq(cas_model.nv,cas_model.nv);
    g_partial_dq.setZero();

    // Call the derivatives function
    pinocchio::computeGeneralizedGravityDerivatives(cas_model, cas_data, q_casadi, g_partial_dq);

    // Get the result into an SX
    pinocchio::casadi::copy( g_partial_dq, g_partial_dq_sx );

    // Create function
    casadi::Function    generalized_gravity_derivatives("generalized_gravity_derivatives", casadi::SXVector {q_sx}, casadi::SXVector {g_partial_dq_sx});

    return generalized_gravity_derivatives;



  }

  casadi::Function get_inverse_dynamics_derivatives(CasadiModel &cas_model, CasadiData &cas_data, std::string type)
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

    // // Output variable
    CasadiScalar          dtau_dq_sx( cas_model.nv, cas_model.nv);
    CasadiData::MatrixXs  dtau_dq_casadi(cas_model.nv,cas_model.nv);
    dtau_dq_casadi.setZero();

    CasadiScalar          dtau_dv_sx( cas_model.nv, cas_model.nv);
    CasadiData::MatrixXs  dtau_dv_casadi(cas_model.nv,cas_model.nv);
    dtau_dv_casadi.setZero();

    CasadiScalar          dtau_da_sx( cas_model.nv, cas_model.nv);
    CasadiData::MatrixXs  dtau_da_casadi(cas_model.nv,cas_model.nv);
    dtau_da_casadi.setZero();

    // Call the derivatives function
    pinocchio::computeRNEADerivatives(cas_model, cas_data, q_casadi, v_casadi, a_casadi);
    cas_data.M.triangularView<Eigen::StrictlyLower>()= cas_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Get the result from analytical derivatives into an SX
    pinocchio::casadi::copy(cas_data.dtau_dq, dtau_dq_sx);
    pinocchio::casadi::copy(cas_data.dtau_dv, dtau_dv_sx);
    pinocchio::casadi::copy(cas_data.M, dtau_da_sx);

    if (type == "dtau_dq") {
      casadi::Function    dtau_dq("dtau_dq", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {dtau_dq_sx}, std::vector<std::string>{"q","v","a"}, std::vector<std::string>{"dtau_dq"});
      return dtau_dq;
    }
    else if (type == "dtau_dv") {
      casadi::Function    dtau_dv("dtau_dv", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {dtau_dv_sx}, std::vector<std::string>{"q","v","a"}, std::vector<std::string>{"dtau_dv"});
      return dtau_dv;
    }
    else if (type == "dtau_da") {
      casadi::Function    dtau_da("dtau_da", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {dtau_da_sx}, std::vector<std::string>{"q","v","a"}, std::vector<std::string>{"dtau_da"});
      return dtau_da;
    }
    else if (type == "jacobian") {
      casadi::Function    rnea_jacobian("rnea_jacobian", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {horzcat(dtau_dq_sx, dtau_dv_sx, dtau_da_sx)}, std::vector<std::string>{"q","v","a"}, std::vector<std::string>{"J_tau"});
      return rnea_jacobian;
    }
    else {
      casadi::Function    rnea_derivatives("rnea_derivatives", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {dtau_dq_sx, dtau_dv_sx, dtau_da_sx}, std::vector<std::string>{"q","v","a"}, std::vector<std::string>{"dtau_dq", "dtau_dv", "dtau_da"});
      return rnea_derivatives;
    }
  }

}
