#include "robot_expressions.hpp"

namespace mecali
{

  casadi::Function get_robot_expressions(CasadiModel &cas_model, CasadiData &cas_data, std::vector<std::string> frame_names)
  {
    std::vector<CasadiScalar> func_outputs;
    std::vector<std::string>  output_names;
    int n_req_frames = frame_names.size();
    int frame_idx;

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

    // ----------------------------------------------------------------------
    // Forward dynamics -----------------------------------------------------
    // ----------------------------------------------------------------------
    // Call Articulated-body algorithm
    pinocchio::aba(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);
    // Get result from ABA into an SX
    CasadiScalar        ddq_sx(cas_model.nv, 1);
    pinocchio::casadi::copy( cas_data.ddq, ddq_sx );

    // fill output vector of the function
    func_outputs.insert(func_outputs.end(), ddq_sx);
    // fill output names vector
    output_names.insert(output_names.end(), "fwd_dyn");

    // ----------------------------------------------------------------------
    // Forward kinematics  --------------------------------------------------
    // ----------------------------------------------------------------------
    // Call forward kinematics function
    pinocchio::forwardKinematics(    cas_model,   cas_data,    q_casadi);
    pinocchio::updateFramePlacements(cas_model,   cas_data);



    CasadiScalar T_sx(4,4);

    for (int i = 0; i < n_req_frames; i++) // Add T for all requested frames
    {
        // get frame index
        frame_idx = cas_model.getFrameId(frame_names[i]);

        // get the result (transformation matrix) from FK
        for(Eigen::DenseIndex i = 0; i < 3; ++i)
        {
          for(Eigen::DenseIndex j = 0; j < 3; ++j)
          {
            T_sx(i,j) = cas_data.oMf[frame_idx].rotation()(i,j);
          }
        }
        T_sx(0,3) = cas_data.oMf[frame_idx].translation()(0);
        T_sx(1,3) = cas_data.oMf[frame_idx].translation()(1);
        T_sx(2,3) = cas_data.oMf[frame_idx].translation()(2);
        T_sx(3,0) = 0;
        T_sx(3,1) = 0;
        T_sx(3,2) = 0;
        T_sx(3,3) = 1;

        // fill output vector of the function
        func_outputs.insert(func_outputs.end(), T_sx);
        // fill output names vector
        output_names.insert(output_names.end(), "T_"+cas_model.frames[frame_idx].name);
    }

    casadi::Function    expressions("expressions", casadi::SXVector {q_sx, v_sx, tau_sx}, func_outputs, std::vector<std::string>{"q", "dq", "tau"}, output_names);

    return expressions;
  }


}
