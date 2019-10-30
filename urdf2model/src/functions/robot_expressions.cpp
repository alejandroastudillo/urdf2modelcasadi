#include "robot_expressions.hpp"

namespace mecali
{

  casadi::Function get_robot_expressions(CasadiModel &cas_model, CasadiData &cas_data, std::vector<std::string> frame_names, bool AUGMENT_ODE)
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

    int n_path_states = 2;
    int n_path_inputs = 1;
    int nx = 2*cas_model.nv + n_path_states;
    int nu = cas_model.nv + n_path_inputs;
    CasadiScalar aug_state_sx = casadi::SX::sym("aug_state", nx);
    CasadiScalar aug_input_sx = casadi::SX::sym("aug_input", nu);
    CasadiScalar aug_output_sx = casadi::SX::sym("aug_output", nx);

    if (AUGMENT_ODE)
    {
      // Path dynamics definition

      CasadiScalar     vp_sx    = casadi::SX::sym("vp", n_path_states);
      CasadiScalar     wp_sx    = casadi::SX::sym("wp", n_path_inputs);
      CasadiScalar     outp_sx  = casadi::SX::sym("outp", n_path_states);
      outp_sx(0) = vp_sx(1);
      outp_sx(1) = wp_sx;
      casadi::Function path_dyn = casadi::Function("path_dyn", casadi::SXVector {vp_sx, wp_sx}, casadi::SXVector {outp_sx});
      // std::cout << path_dyn(casadi::DMVector {std::vector<double>{1,2},3})[0] << std::endl;

      // Augmented dynamics
      for(int j = 0; j < cas_model.nv; ++j)             { aug_output_sx(j) = v_sx(j); }
      for(int j = cas_model.nv; j < 2*cas_model.nv; ++j){ aug_output_sx(j) = ddq_sx(j-cas_model.nv); }
      for(int j = 2*cas_model.nv; j < nx; ++j)          { aug_output_sx(j) = outp_sx(j-(2*cas_model.nv)); }
      // std::cout << aug_output_sx << std::endl;



      for(int j = 0; j < cas_model.nv; ++j)             { aug_state_sx(j) = q_sx(j); }
      for(int j = cas_model.nv; j < 2*cas_model.nv; ++j){ aug_state_sx(j) = v_sx(j-cas_model.nv); }
      for(int j = 2*cas_model.nv; j < nx; ++j)          { aug_state_sx(j) = vp_sx(j-(2*cas_model.nv)); }

      for(int j = 0; j < cas_model.nv; ++j)             { aug_input_sx(j) = tau_sx(j); }
      for(int j = cas_model.nv; j < nu; ++j)            { aug_input_sx(j) = wp_sx(j-cas_model.nv); }
      // std::cout << aug_state_sx << std::endl;
      // std::cout << aug_input_sx << std::endl;

      // casadi::Function ode_aug = casadi::Function("ode_aug", casadi::SXVector {q_sx, v_sx, tau_sx, vp_sx, wp_sx}, casadi::SXVector {aug_output_sx});
      // casadi::Function ode_aug = casadi::Function("ode_aug", casadi::SXVector {aug_state_sx, aug_input_sx}, casadi::SXVector {aug_output_sx});
      // std::cout << ode_aug << std::endl;

      // fill output vector of the function
      func_outputs.insert(func_outputs.end(), aug_output_sx);
      // fill output names vector
      output_names.insert(output_names.end(), "ode_aug");


    } else
    {
      // fill output vector of the function
      func_outputs.insert(func_outputs.end(), ddq_sx);
      // fill output names vector
      output_names.insert(output_names.end(), "fwd_dyn");
    }



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


    if (AUGMENT_ODE)
    {
      casadi::Function    expressions("expressions", casadi::SXVector {aug_state_sx, aug_input_sx}, func_outputs, std::vector<std::string>{"aug_state", "aug_input"}, output_names);
      return expressions;
    } else
    {
      casadi::Function    expressions("expressions", casadi::SXVector {q_sx, v_sx, tau_sx}, func_outputs, std::vector<std::string>{"q", "dq", "tau"}, output_names);
      return expressions;
    }


// TODO: Compare ode_aug from here, with old ode_aug in Matlab. Then simplify sx.
  }


}
