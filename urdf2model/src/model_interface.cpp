/* Check:
https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html
https://coin-or.github.io/CppAD/doc/cppad_eigen.hpp.htm

### Pinocchio doc:
https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio.html#a97be7e9cd332a591b1431e30bab2c51e

### Information about types of variables
Eigen::VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1> = pinocchio::ModelTpl<double>::VectorXs
Eigen::Vector3d = Eigen::Matrix<double, 3, 1> = pinocchio::ModelTpl<double>::Vector3

  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, Eigen::Dynamic>  EigenCasadiMatrix;
  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, 1>               EigenCasadiVecXd;
  typedef Eigen::Matrix<CasadiScalar , 3, 1>                            EigenCasadiVec3d;
*/

/* TODO Handle (print error or warning) when the torque, position, or velocity limits are zero.
   TODO Add some option to simplify the input of the functions (for instance instead of q[11] use q[7] for Kinova)
*/

#include "model_interface.hpp"

#include "utils/debug_functions.hpp"

namespace mecali
{
  const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;

  Serial_Robot::Serial_Robot(void)
  {
    name = "NOT_SET";
  }

  void              Serial_Robot::import_model(std::string filename, bool verbose)
  {
    // Pinocchio model
      Model         model;
    // Build the model using the URDF parser
      pinocchio::urdf::buildModel(filename,model,verbose);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
    // Set the gravity applied to the model
      model.gravity.linear(pinocchio::Model::gravity981);
    // Initialize the data structure for the model
      Data          data = pinocchio::Data(model);

    // populate the data structure with some basic information about the robot
      this->name                  = model.name;
      this->n_joints              = model.njoints;  // data.oMi.size()
      this->n_frames              = model.nframes;
      this->n_q                   = model.nq;
      this->n_dof                 = model.nv;
      this->gravity               = model.gravity.linear_impl();
      this->joint_torque_limit    = model.effortLimit;
      this->joint_pos_ub          = model.upperPositionLimit;
      this->joint_pos_lb          = model.lowerPositionLimit;
      this->joint_vel_limit       = model.velocityLimit;
      this->joint_names           = model.names;
      this->neutral_configuration = pinocchio::neutral(model);

      this->_n_bodies             = model.nbodies;
      this->_model                = model;

      std::vector<std::string> joint_types(this->n_dof);
      for (int i = 1; i < this->n_joints; i++){ joint_types[i-1] = model.joints[i].shortname(); }
      this->joint_types           = joint_types;

    // Casadi model
      CasadiModel casadi_model = model.cast<CasadiScalar>();
      CasadiData casadi_data( casadi_model );

      this->_casadi_model         = casadi_model;
  }
  void              Serial_Robot::import_model(std::string filename)
  {
    this->import_model(filename, false); // verbose can be omitted from the buildModel execution: <pinocchio::urdf::buildModel(filename,model)>
  }

  Eigen::VectorXd   Serial_Robot::randomConfiguration()
  {
    Eigen::VectorXd lb_model = this->joint_pos_lb;
    Eigen::VectorXd ub_model = this->joint_pos_ub; // In this function, lb and ub size = n_q

    // Check for continuous joints
    if (this->n_q > this->n_dof) // If the length of the configuration-vector is > than the DoF, some joints are continuous and represented by [cos(q_j) sin(q_j)]
    {
      // Create a boundary vectors of size n_q, filled with zeros.
      Eigen::VectorXd lb_dof = Eigen::VectorXd::Zero(this->n_dof, 1);
      Eigen::VectorXd ub_dof = Eigen::VectorXd::Zero(this->n_dof, 1);

      // Creates an index for the boundary vectors
      int j = 0;
      // Iterates for all the components of the boundary vectors.
      for (Eigen::DenseIndex k = 0; k < this->n_dof; ++k)
      {
        // Check if joint k is a revolute unbounded (continuous) joint (These joint types come from Pinocchio)
        if (this->joint_types[k].compare("JointModelRUBZ") == 0 || this->joint_types[k].compare("JointModelRUBY") == 0 || this->joint_types[k].compare("JointModelRUBX") == 0)
        {
          // Fill the corresponding value in the boundary vectors
          lb_dof[k] = -PI;
          ub_dof[k] = PI;
          // Update the configuration vector index.
          j = j+2;
        } else // If joint k is not a continuous joint
        {
          // Fill the corresponding value in the boundary vectors
          lb_dof[k] = lb_model[j];
          ub_dof[k] = ub_model[j];
          // Update the configuration vector index.
          j++;
        }
      }
      return this->randomConfiguration(lb_dof, ub_dof);
    } else // There is no continuous joint in this robot
    {
      return this->randomConfiguration(lb_model, ub_model);
    }
  }
  Eigen::VectorXd   Serial_Robot::randomConfiguration(Eigen::VectorXd lower_bounds, Eigen::VectorXd upper_bounds)
  {
    // QUESTION Should these lower and upper bounds be compared with this->joint_pos_lb and this->joint_pos_ub?

    // Assert that both the lower_bounds and upper_bounds vectors are of length equal to n_dof.
    custom_assert(lower_bounds.size() == this->n_dof && upper_bounds.size() == this->n_dof, "Error in " + std::string(__FUNCTION__) + "(): Lower and upper bound vectors must be of length equal to n_dof.");

    for (int i = 0; i < this->n_dof; ++i)
    {
      custom_assert(lower_bounds[i] <= upper_bounds[i], "Error in " + std::string(__FUNCTION__) + "(): Lower bound [" + std::to_string(i) + "] must be lower than upper bound [" + std::to_string(i) + "]");
    }

    Eigen::VectorXd lb = lower_bounds;
    Eigen::VectorXd ub = upper_bounds;

    Eigen::VectorXd randConfig;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 mt_rand(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> rnd_dis(-1.0, 1.0);

    // Check for continuous joints
    if (this->n_q > this->n_dof) // If the length of the configuration-vector is > than the DoF, some joints are continuous and represented by [cos(q_j) sin(q_j)]
    {
      // Create a vector of size = n_dof. (This will be filled with random numbers between lb and ub).
      Eigen::VectorXd randAngles = Eigen::VectorXd::Zero(this->n_dof, 1);
      // Create a configuration vector of size n_q, filled with zeros.
      randConfig = Eigen::VectorXd::Zero(this->n_q);
      // Creates an index for the configuration vector
      int j = 0;
      // Iterates for all the components of the random angles vector.
      for (Eigen::DenseIndex k = 0; k < this->n_dof; ++k)
      {
        // Use mt19937 to set a random number between -1 and 1.
        randAngles[k] = rnd_dis(mt_rand);
        // Adjust the range of the random values from [-1, 1] to [ub, lb].
        if (randAngles[k] < 0) { randAngles[k] = -1*lb[k]*randAngles[k]; }
        else { randAngles[k] = ub[k]*randAngles[k]; }
        // Check if joint k is a revolute unbounded (continuous) joint (This joint types come from Pinocchio)
        if (this->joint_types[k].compare("JointModelRUBZ") == 0 || this->joint_types[k].compare("JointModelRUBY") == 0 || this->joint_types[k].compare("JointModelRUBX") == 0)
        {
          // Fill the corresponding values in the configuration vector.
          randConfig[j] = cos(randAngles[k]);
          randConfig[j+1] = sin(randAngles[k]);
          // Update the configuration vector index.
          j = j+2;
        } else // If joint k is not a continuous joint
        {
          // Directly fill the corresponding value in the configuration vector
          randConfig[j] = randAngles[k];
          // Update the configuration vector index.
          j++;
        }
      }
    } else // If there is no continuous joint in this robot
    {
      // Create a configuration vector of size = n_q.
      randConfig = Eigen::VectorXd::Zero(this->n_q, 1);

      for (Eigen::DenseIndex k = 0; k < this->n_q; ++k)
      {
        // Use mt19937 to set a random number between -1 and 1.
        randConfig[k] = rnd_dis(mt_rand);
        // Adjust the range of the random values from [-1, 1] to [ub, lb].
        if (randConfig[k] < 0) { randConfig[k] = -1*lb[k]*randConfig[k]; }
        else { randConfig[k] = ub[k]*randConfig[k]; }
      }
    }
    return randConfig;
  }
  Eigen::VectorXd   Serial_Robot::randomConfiguration(std::vector<double> lower_bounds_v, std::vector<double> upper_bounds_v)
  {
    // Assert that both the lower_bounds and upper_bounds vectors are of length equal to n_dof.
    custom_assert(lower_bounds_v.size() == this->n_dof && upper_bounds_v.size() == this->n_dof, "Error in " + std::string(__FUNCTION__) + "(): Lower and upper bound vectors must be of length equal to n_dof.");

    Eigen::Map<Eigen::VectorXd> lb(&lower_bounds_v[0], this->n_dof);
    Eigen::Map<Eigen::VectorXd> ub(&upper_bounds_v[0], this->n_dof);

    return this->randomConfiguration(lb, ub);
  }

  casadi::Function  Serial_Robot::forward_dynamics()
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_forward_dynamics( this->_casadi_model, casadi_data );
  }
  casadi::Function  Serial_Robot::inverse_dynamics()
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_inverse_dynamics( this->_casadi_model, casadi_data );
  }
  casadi::Function  Serial_Robot::generalized_gravity()
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_generalized_gravity( this->_casadi_model, casadi_data );
  }
  casadi::Function  Serial_Robot::coriolis_matrix()
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_coriolis( this->_casadi_model, casadi_data );
  }
  casadi::Function  Serial_Robot::mass_inverse_matrix()
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_mass_inverse( this->_casadi_model, casadi_data );
  }

  casadi::Function  Serial_Robot::forward_kinematics(std::string content, std::vector<std::string> frame_names)
  {
      CasadiData casadi_data( this->_casadi_model );

      return get_forward_kinematics(this->_casadi_model, casadi_data, content, frame_names);
  }
  casadi::Function  Serial_Robot::forward_kinematics(std::string content, std::vector<int> frame_indices)
  {
      std::vector<std::string> req_frame_names;
      for (int i = 0; i < frame_indices.size(); i++)
      {
          req_frame_names.insert(req_frame_names.end(), this->_model.frames[frame_indices[i]].name);
      }
      return this->forward_kinematics(content, req_frame_names);
  }
  casadi::Function  Serial_Robot::forward_kinematics(std::string content, std::string frame_name)
  {
      return this->forward_kinematics(content, std::vector<std::string>{frame_name});
  }
  casadi::Function  Serial_Robot::forward_kinematics(std::string content, int frame_index)
  {
      return this->forward_kinematics(content, std::vector<int>{frame_index});
  }
  casadi::Function  Serial_Robot::forward_kinematics(std::string content)
  {
      std::vector<std::string> all_frame_names;
      // Frames "universe" and "root_joint" are not taken into account (they are added by Pinocchio, not from URDF)
      for (int i = 2; i < this->n_frames; i++)
      {
          all_frame_names.insert(all_frame_names.end(), this->_model.frames[i].name);
      }

      return this->forward_kinematics(content, all_frame_names);
  }
  casadi::Function  Serial_Robot::forward_kinematics()
  {
      return this->forward_kinematics("transformation");
  }

  void              Serial_Robot::print_model_data()
  {
      std::cout << "\n----- Robot model information: " << std::endl;
      print_indent("Model name = ",                         this->name,               38);
      print_indent("Size of configuration vector = ",       this->n_q,                38);
      print_indent("Number of joints (with universe) = ",   this->n_joints,           38);
      print_indent("Number of DoF = ",                      this->n_dof,              38);
      print_indent("Number of bodies = ",                   this->_n_bodies,          38);
      print_indent("Number of operational frames = ",       this->n_frames,           38);
      print_indent("Gravity = ",                            this->gravity,            38);
      print_indent("Joint torque bounds = ",                this->joint_torque_limit, 38);
      print_indent("Joint configuration upper bounds = ",   this->joint_pos_ub,       38);
      print_indent("Joint configuration lower bounds = ",   this->joint_pos_lb,       38);
      print_indent("Joint velocity bounds = ",              this->joint_vel_limit,    38);
      std::cout << std::endl;

      // std::cout << "\n----- Placement of each joint in the model: " << std::endl;
      std::cout << "-----Name of each joint in the model: " << std::endl;
      for (int k=0 ; k<this->n_joints ; ++k)
      {
          std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << this->joint_names[k] << std::setw(10) << std::endl; // << data.oMi[k].translation().transpose()
      }

      // std::cout << "\n----- Name of each frame in the model: " << std::endl;
      // for (int k=0 ; k<this->n_frames ; ++k)
      // {
      //     std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << this->_model.frames[k].name << std::endl;
      // }

      Data _data = pinocchio::Data(this->_model);
      pinocchio::forwardKinematics(this->_model, _data, this->neutral_configuration);
      pinocchio::updateFramePlacements(this->_model, _data);
      std::cout << "\n----- Placement of each frame in the model (in neutral configuration): " << std::endl;
      for (int k=0 ; k<this->n_frames ; ++k)
      {
          std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << _model.frames[k].name << std::setw(10) << _data.oMf[k].translation().transpose() << std::endl;
      }
      std::cout << std::endl;
  }

}

extern "C" {
    mecali::Serial_Robot* Serial_Robot_new(){ return new mecali::Serial_Robot; }
    void import_model_new(mecali::Serial_Robot* robot_model, std::string filename){ robot_model->import_model(filename);}
    // void SR_import_model(Serial_Robot* robot_model){ robot_model->import_model(); }
    std::string name_new(mecali::Serial_Robot* robot_model){ return robot_model->name; }
}
//       // Copy Casadi to EIGEN
//       // casadi::SX cs_mat = casadi::SX::sym("A", 3, 4);
//       // Eigen::Matrix<casadi::SX, 3, 4> eig_mat;
//       //
//       // pinocchio::casadi::copy(cs_mat, eig_mat);
//       // std::cout << eig_mat << std::endl;
//
//       // Copy EIGEN to Casadi
//       // Eigen::Matrix<casadi::SX, 3, 4> eig_mat2;
//       // pinocchio::casadi::sym(eig_mat2, "A");
//       //
//       // casadi::SX cs_mat2;
//       //
//       // pinocchio::casadi::copy(eig_mat2, cs_mat2);
//       // std::cout << cs_mat2 << std::endl;


      // CasadiData casadi_data( this->_casadi_model );
      // int        n_req_frames = frame_names.size();
      //
      // std::vector<CasadiScalar> func_outputs;
      //
      // std::vector<std::string>  output_names;
      //
      // // create the input vector (for pinocchio and for the returned function)
      // CasadiScalar        q_sx = casadi::SX::sym("q", this->_casadi_model.nq);
      // ConfigVectorCasadi  q_casadi(this->_casadi_model.nq);
      // pinocchio::casadi::copy( q_sx, q_casadi );
      //
      // // call the forward kinematics function
      // pinocchio::forwardKinematics(    this->_casadi_model,   casadi_data,    q_casadi);
      // pinocchio::updateFramePlacements(this->_casadi_model,   casadi_data);
      //
      // int frame_idx;
      //
      // if (content.compare("position") == 0)
      // {
      //     CasadiScalar  pos_sx(3,1);
      //
      //     for (int i = 0; i < n_req_frames; i++)
      //     {
      //         // get frame index
      //         frame_idx = this->_casadi_model.getFrameId(frame_names[i]);
      //         // std::cout << "The frame with name: " << frame_names[i] << " has an index number: " << frame_idx << std::endl;
      //
      //         // get the result (translation vector) from FK
      //         pinocchio::casadi::copy( casadi_data.oMf[frame_idx].translation(), pos_sx );
      //
      //         // fill the output vector of the function
      //         func_outputs.insert(func_outputs.end(), pos_sx);
      //
      //         // fill the output names vector
      //         output_names.insert(output_names.end(), "Pos_"+this->_model.frames[frame_idx].name);
      //     }
      //
      //     return casadi::Function( "fk_pos", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);
      // }
      // else if (content.compare("rotation") == 0)
      // {
      //     CasadiScalar rot_sx(3,3);
      //
      //     for (int i = 0; i < n_req_frames; i++)
      //     {
      //         // get frame index
      //         frame_idx = this->_casadi_model.getFrameId(frame_names[i]);
      //
      //         // get the result (rotation matrix) from FK
      //         pinocchio::casadi::copy( casadi_data.oMf[frame_idx].rotation(), rot_sx );
      //
      //         // fill the output vector of the function
      //         func_outputs.insert(func_outputs.end(), rot_sx);
      //
      //         // fill the output names vector
      //         output_names.insert(output_names.end(), "Rot_"+this->_model.frames[frame_idx].name);
      //     }
      //
      //     return casadi::Function( "fk_rot", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);
      //
      // }
      // else if (content.compare("transformation") == 0)
      // {
      //     CasadiScalar T_sx(4,4);
      //
      //     for (int i = 0; i < n_req_frames; i++)
      //     {
      //         // get frame index
      //         frame_idx = this->_casadi_model.getFrameId(frame_names[i]);
      //
      //         // get the result (transformation matrix) from FK
      //         for(Eigen::DenseIndex i = 0; i < 3; ++i)
      //         {
      //           for(Eigen::DenseIndex j = 0; j < 3; ++j)
      //           {
      //             T_sx(i,j) = casadi_data.oMf[frame_idx].rotation()(i,j);
      //           }
      //         }
      //         T_sx(0,3) = casadi_data.oMf[frame_idx].translation()(0);
      //         T_sx(1,3) = casadi_data.oMf[frame_idx].translation()(1);
      //         T_sx(2,3) = casadi_data.oMf[frame_idx].translation()(2);
      //         T_sx(3,0) = 0;
      //         T_sx(3,1) = 0;
      //         T_sx(3,2) = 0;
      //         T_sx(3,3) = 1;
      //
      //         // fill the output vector of the function
      //         func_outputs.insert(func_outputs.end(), T_sx);
      //
      //         // fill the output names vector
      //         output_names.insert(output_names.end(), "T_"+this->_model.frames[frame_idx].name);
      //     }
      //
      //     return casadi::Function( "fk_T", casadi::SXVector {q_sx}, func_outputs, std::vector<std::string>{"q"}, output_names);
      // }
      // else
      // {
      //     throw std::invalid_argument("There is no option \'" + content + "\' for type of forward kinematics");
      // }
