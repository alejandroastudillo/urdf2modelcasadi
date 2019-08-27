#include "forward_kinematics.hpp"

namespace mecali
{
  casadi::Function get_forward_kinematics(CasadiModel &cas_model, CasadiData &cas_data, std::string content, std::vector<std::string> frame_names)
  {
    int        n_req_frames = frame_names.size();

    std::vector<CasadiScalar> func_outputs;

    std::vector<std::string>  output_names;

    // create the input vector (for pinocchio and for the returned function)
    CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    ConfigVectorCasadi  q_casadi(cas_model.nq);
    pinocchio::casadi::copy( q_sx, q_casadi );

    // call the forward kinematics function
    pinocchio::forwardKinematics(    cas_model,   cas_data,    q_casadi);
    pinocchio::updateFramePlacements(cas_model,   cas_data);

    int frame_idx;

    if (content.compare("position") == 0)
    {
        CasadiScalar  pos_sx(3,1);

        for (int i = 0; i < n_req_frames; i++)
        {
            // get frame index
            frame_idx = cas_model.getFrameId(frame_names[i]);
            // std::cout << "The frame with name: " << frame_names[i] << " has an index number: " << frame_idx << std::endl;

            // get the result (translation vector) from FK
            pinocchio::casadi::copy( cas_data.oMf[frame_idx].translation(), pos_sx );

            // fill the output vector of the function
            func_outputs.insert(func_outputs.end(), pos_sx);

            // fill the output names vector
            output_names.insert(output_names.end(), "Pos_"+cas_model.frames[frame_idx].name);
        }

        return casadi::Function( "fk_pos", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);
    }
    else if (content.compare("rotation") == 0)
    {
        CasadiScalar rot_sx(3,3);

        for (int i = 0; i < n_req_frames; i++)
        {
            // get frame index
            frame_idx = cas_model.getFrameId(frame_names[i]);

            // get the result (rotation matrix) from FK
            pinocchio::casadi::copy( cas_data.oMf[frame_idx].rotation(), rot_sx );

            // fill the output vector of the function
            func_outputs.insert(func_outputs.end(), rot_sx);

            // fill the output names vector
            output_names.insert(output_names.end(), "Rot_"+cas_model.frames[frame_idx].name);
        }

        return casadi::Function( "fk_rot", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);

    }
    else if (content.compare("transformation") == 0)
    {
        CasadiScalar T_sx(4,4);

        for (int i = 0; i < n_req_frames; i++)
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

            // fill the output vector of the function
            func_outputs.insert(func_outputs.end(), T_sx);

            // fill the output names vector
            output_names.insert(output_names.end(), "T_"+cas_model.frames[frame_idx].name);
        }

        return casadi::Function( "fk_T", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);
    }
    else
    {
        throw std::invalid_argument("There is no option \'" + content + "\' for type of forward kinematics");
    }
  }
}

// casadi::Function get_forward_kinematics_position(CasadiModel &cas_model, CasadiData &cas_data, std::string frame_name)
// {
//   int                 EE_idx = cas_model.getFrameId(frame_name);
//   std::cout << "The frame with name: " << frame_name << " has an index of: " << EE_idx << std::endl;
//   // Set variables
//   CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
//   ConfigVectorCasadi  q_casadi(cas_model.nq);
//   pinocchio::casadi::copy( q_sx, q_casadi );
//
//   // Call the forward kinematics function
//   pinocchio::forwardKinematics(     cas_model,   cas_data,    q_casadi);
//   pinocchio::updateFramePlacements( cas_model,   cas_data);
//
//   // Get the result from FK
//   CasadiScalar        pos_sx(3,1);
//   pinocchio::casadi::copy( cas_data.oMf[EE_idx].translation(), pos_sx );
//
//   // Create the Forward Kinematics function
//   casadi::Function    fk_pos( "fk_pos", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx} );
//
//   return fk_pos;
// }
// casadi::Function get_forward_kinematics_position(CasadiModel &cas_model, CasadiData &cas_data)
// {
//   int                 EE_idx = cas_model.nframes-1;
//   // Set variables
//   CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
//   ConfigVectorCasadi  q_casadi(cas_model.nq);
//   pinocchio::casadi::copy( q_sx, q_casadi );
//
//   // Call the forward kinematics function
//   pinocchio::forwardKinematics(     cas_model,   cas_data,    q_casadi);
//   pinocchio::updateFramePlacements( cas_model,   cas_data);
//
//   // Get the result from FK
//   CasadiScalar        pos_sx(3,1);
//   pinocchio::casadi::copy( cas_data.oMf[EE_idx].translation(), pos_sx );
//
//   // Create the Forward Kinematics function
//   casadi::Function    fk_pos( "fk_pos", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx} );
//
//   return fk_pos;
// }
// casadi::Function get_forward_kinematics_rotation(CasadiModel &cas_model, CasadiData &cas_data, std::string frame_name)
// {
//   int                 EE_idx = cas_model.getFrameId(frame_name);
//   std::cout << "The frame with name: " << frame_name << " has an index of: " << EE_idx << std::endl;
//   // Set variables
//   CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
//   ConfigVectorCasadi  q_casadi(cas_model.nq);
//   pinocchio::casadi::copy( q_sx, q_casadi );
//
//   // Call the forward kinematics function
//   pinocchio::forwardKinematics(     cas_model,   cas_data,    q_casadi);
//   pinocchio::updateFramePlacements( cas_model,   cas_data);
//
//   // Get the result from FK
//   CasadiScalar        rot_sx(3,3);
//   for(Eigen::DenseIndex i = 0; i < 3; ++i)
//   {
//     for(Eigen::DenseIndex j = 0; j < 3; ++j)
//     {
//       rot_sx(i,j) = cas_data.oMf[EE_idx].rotation()(i,j);
//     }
//   }
//
//   // Create the Forward Kinematics function
//   casadi::Function    fk_rot( "fk_rot", casadi::SXVector {q_sx}, casadi::SXVector {rot_sx} );
//
//   return fk_rot;
// }
// casadi::Function get_forward_kinematics_rotation(CasadiModel &cas_model, CasadiData &cas_data)
// {
//   int                 EE_idx = cas_model.nframes-1;
//   // Set variables
//   CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
//   ConfigVectorCasadi  q_casadi(cas_model.nq);
//   pinocchio::casadi::copy( q_sx, q_casadi );
//
//   // Call the forward kinematics function
//   pinocchio::forwardKinematics(     cas_model,   cas_data,    q_casadi);
//   pinocchio::updateFramePlacements( cas_model,   cas_data);
//
//   // Get the result from FK
//   CasadiScalar        rot_sx(3,3);
//   for(Eigen::DenseIndex i = 0; i < 3; ++i)
//   {
//     for(Eigen::DenseIndex j = 0; j < 3; ++j)
//     {
//       rot_sx(i,j) = cas_data.oMf[EE_idx].rotation()(i,j);
//     }
//   }
//
//   // Create the Forward Kinematics function
//   casadi::Function    fk_rot( "fk_rot", casadi::SXVector {q_sx}, casadi::SXVector {rot_sx} );
//
//   return fk_rot;
// }
