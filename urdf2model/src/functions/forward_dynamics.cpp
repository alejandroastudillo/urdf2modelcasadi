#include "forward_dynamics.hpp"

namespace mecali
{

  casadi::Function get_forward_dynamics(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),cas_model.nq,1);
    // pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi(cas_model.nv);
    v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),cas_model.nv,1);
    // pinocchio::casadi::copy( v_sx, v_casadi ); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

    CasadiScalar        tau_sx = casadi::SX::sym("tau", cas_model.nv);
    TangentVectorCasadi tau_casadi(cas_model.nv);
    tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),cas_model.nv,1);
    // pinocchio::casadi::copy( tau_sx, tau_casadi ); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

    // Call the Articulated-body algorithm
    pinocchio::aba(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);

    // Get the result from ABA into an SX
    CasadiScalar        ddq_sx(cas_model.nv, 1);
    for(Eigen::DenseIndex k = 0; k < cas_model.nv; ++k)
      ddq_sx(k) = cas_data.ddq[k];
    // pinocchio::casadi::copy( cas_data.ddq, ddq_sx );

    // Create the ABA function
    casadi::Function    aba("aba", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_sx}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"ddq"});

    return aba;
  }

  casadi::Function get_mass_inverse(CasadiModel &cas_model, CasadiData &cas_data)
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi(cas_model.nv);
    pinocchio::casadi::copy( v_sx, v_casadi ); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

    CasadiScalar        tau_sx = casadi::SX::sym("tau", cas_model.nv);
    TangentVectorCasadi tau_casadi(cas_model.nv);
    pinocchio::casadi::copy( tau_sx, tau_casadi ); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

    // Call the Articulated-body algorithm
    pinocchio::aba(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);
    pinocchio::computeMinverse(cas_model, cas_data, q_casadi);

    // TODO: There should be a more efficient way to do this:
    cas_data.Minv.triangularView<Eigen::StrictlyLower>() = cas_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

    // Get the result from ABA into an SX
    CasadiScalar        Minv_sx(cas_model.nv, cas_model.nv);
    pinocchio::casadi::copy( cas_data.Minv, Minv_sx );

    // Create the ABA function
    casadi::Function    mass_inverse("Minv", casadi::SXVector {q_sx}, casadi::SXVector {Minv_sx});

    return mass_inverse;
  }

  casadi::Function get_forward_dynamics_derivatives(CasadiModel &cas_model, CasadiData &cas_data, std::string type )
  {
    // Set variables
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
    TangentVectorCasadi v_casadi(cas_model.nv);
    pinocchio::casadi::copy( v_sx, v_casadi ); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

    CasadiScalar        tau_sx = casadi::SX::sym("tau", cas_model.nv);
    TangentVectorCasadi tau_casadi(cas_model.nv);
    pinocchio::casadi::copy( tau_sx, tau_casadi ); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

    // Call the derivatives from Pinocchio
    pinocchio::computeABADerivatives(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);
    cas_data.Minv.triangularView<Eigen::StrictlyLower>() = cas_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

    // Get the result from Pinocchio into an SX
    CasadiScalar          ddq_dq_sx( cas_model.nv, cas_model.nv);
    CasadiScalar          ddq_dv_sx( cas_model.nv, cas_model.nv);
    CasadiScalar          ddq_dtau_sx( cas_model.nv, cas_model.nv);
    pinocchio::casadi::copy(cas_data.ddq_dq, ddq_dq_sx);
    pinocchio::casadi::copy(cas_data.ddq_dv, ddq_dv_sx);
    pinocchio::casadi::copy(cas_data.Minv, ddq_dtau_sx);

    // Create the Casadi function
    if (type == "ddq_dq") {
      casadi::Function    ddq_dq("ddq_dq", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_dq_sx}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"ddq_dq"});
      return ddq_dq;
    }
    else if (type == "ddq_dv") {
      casadi::Function    ddq_dv("ddq_dv", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_dv_sx}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"ddq_dv"});
      return ddq_dv;
    }
    else if (type == "ddq_dtau") {
      casadi::Function    ddq_dtau("ddq_dtau", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_dtau_sx}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"ddq_dtau"});
      return ddq_dtau;
    }
    else if (type == "jacobian") {
      casadi::Function    aba_jacobian("aba_jacobian", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {horzcat(ddq_dq_sx, ddq_dv_sx, ddq_dtau_sx)}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"J_ddq"});
      return aba_jacobian;
    }
    else {
      casadi::Function    aba_derivatives("aba_derivatives", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_dq_sx, ddq_dv_sx, ddq_dtau_sx}, std::vector<std::string>{"q","v","tau"}, std::vector<std::string>{"ddq_dq", "ddq_dv", "ddq_dtau"});
      return aba_derivatives;
    }

  }
}
