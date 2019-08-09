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
   TODO In randomConfiguration, assert that each ub[k] >= lb[k] (kind of implemented already) Check if there is a better way
*/

#include "model_interface.hpp"

#include "utils/debug_functions.hpp"

namespace mecali
{
  // const double PI = boost::math::constants::pi<double>();
  const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;

  void Serial_Robot::import_model(std::string filename)
  {
      // Pinocchio model
        Model         model;
      // Build the model using the URDF parser
        bool          verbose = false;                                   // verbose can be omitted from the buildModel execution: <pinocchio::urdf::buildModel(filename,model)>
        pinocchio::urdf::buildModel(filename,model,verbose);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
      // Set the gravity applied to the model
        model.gravity.linear(pinocchio::Model::gravity981);
      // Initialize the data structure for the model
        Data          data = pinocchio::Data(model);

      // populate the data structure with some basic information about the robot
        this->name                  = model.name;
        this->n_joints              = model.njoints;  // data.oMi.size()
        this->n_q                   = model.nq;
        this->n_dof                 = model.nv;
        this->n_bodies              = model.nbodies;
        this->n_frames              = model.nframes;
        this->gravity               = model.gravity.linear_impl();
        this->joint_torque_limit    = model.effortLimit;
        this->joint_pos_ub          = model.upperPositionLimit;
        this->joint_pos_lb          = model.lowerPositionLimit;
        this->joint_vel_limit       = model.velocityLimit;
        this->joint_names           = model.names;
        this->neutral_configuration = pinocchio::neutral(model);

        std::vector<std::string> joint_types(this->n_dof);
        for (int i = 1; i < this->n_joints; i++){ joint_types[i-1] = model.joints[i].shortname(); }
        this->joint_types           = joint_types;

      // Casadi model
        CasadiModel casadi_model = model.cast<CasadiScalar>();
        CasadiData casadi_data( casadi_model );

      // Get functions and populate data structure
        this->aba     = get_forward_dynamics( casadi_model, casadi_data );
        this->rnea    = get_inverse_dynamics( casadi_model, casadi_data );
        this->fk_pos  = get_forward_kinematics_position( casadi_model, casadi_data );
        this->fk_rot  = get_forward_kinematics_rotation( casadi_model, casadi_data );
  }

  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model)
  {
    Eigen::VectorXd lb = rob_model.joint_pos_lb;
    Eigen::VectorXd ub = rob_model.joint_pos_ub; // In this function, lb and ub size = n_q

    Eigen::VectorXd randConfig;

    // std::mt19937 mt_rand(time(0));
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 mt_rand(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> rnd_dis(-1.0, 1.0);

    // Check for continuous joints
    if (rob_model.n_q > rob_model.n_dof) // If the length of the configuration-vector is > than the DoF, some joints are continuous and represented by [cos(q_j) sin(q_j)]
    {
      // Create a random vector of size = n_dof. (This is filled with numbers between -1 and 1).
      Eigen::VectorXd randAngles = Eigen::VectorXd::Zero(rob_model.n_dof, 1);
      // Create a configuration vector of size n_q, filled with zeros.
      randConfig = Eigen::VectorXd::Zero(rob_model.n_q);
      // Creates an index for the configuration vector
      int j = 0;
      // Iterates for all the components of the random angles vector.
      for (Eigen::DenseIndex k = 0; k < rob_model.n_dof; ++k)
      {
        // Check if joint k is a revolute unbounded (continuous) joint (This joint types come from Pinocchio)
        if (rob_model.joint_types[k].compare("JointModelRUBZ") == 0 || rob_model.joint_types[k].compare("JointModelRUBY") == 0 || rob_model.joint_types[k].compare("JointModelRUBX") == 0)
        {
          // Use mt19937 to set a random number in [-1, 1], then multiply it by PI to have [-PI, PI] (Here you can have bounds < or > than PI because it is a continuous joint.
          randAngles[k] = PI*rnd_dis(mt_rand);
          // Fill the corresponding values in the configuration vector.
          randConfig[j] = cos(randAngles[k]);
          randConfig[j+1] = sin(randAngles[k]);
          // Update the configuration vector index.
          j = j+2;
        } else // If joint k is not a continuous joint
        {
          // Use mt19937 to set a random number between -1 and 1.
          randAngles[k] = rnd_dis(mt_rand);
          // Adjust the range of the random values from [-1, 1] to [ub, lb].
          if (randAngles[k] < 0) { randAngles[k] = -1*lb[j]*randAngles[k]; }
          else { randAngles[k] = ub[j]*randAngles[k]; }
          // Fill the corresponding value in the configuration vector
          randConfig[j] = randAngles[k];
          // Update the configuration vector index.
          j++;
        }
      }

    } else // There is no continuous joint in this robot
    {
      // Create a random configuration vector of size = n_q. (This is filled with numbers between -1 and 1).
      randConfig = Eigen::VectorXd::Zero(rob_model.n_q, 1);

      for (Eigen::DenseIndex k = 0; k < rob_model.n_q; ++k)
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
  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model, Eigen::VectorXd lower_bounds, Eigen::VectorXd upper_bounds)
  {
    // QUESTION Should these lower and upper bounds be compared with rob_model.joint_pos_lb and rob_model.joint_pos_ub?

    // Assert that both the lower_bounds and upper_bounds vectors are of length equal to n_dof.
    custom_assert(lower_bounds.size() == rob_model.n_dof && upper_bounds.size() == rob_model.n_dof, "Error in " + std::string(__FUNCTION__) + "(): Lower and upper bound vectors must be of length equal to n_dof.");

    for (int i = 0; i < rob_model.n_dof; ++i)
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
    if (rob_model.n_q > rob_model.n_dof) // If the length of the configuration-vector is > than the DoF, some joints are continuous and represented by [cos(q_j) sin(q_j)]
    {
      // Create a vector of size = n_dof. (This will be filled with random numbers between lb and ub).
      Eigen::VectorXd randAngles = Eigen::VectorXd::Zero(rob_model.n_dof, 1);
      // Create a configuration vector of size n_q, filled with zeros.
      randConfig = Eigen::VectorXd::Zero(rob_model.n_q);
      // Creates an index for the configuration vector
      int j = 0;
      // Iterates for all the components of the random angles vector.
      for (Eigen::DenseIndex k = 0; k < rob_model.n_dof; ++k)
      {
        // Use mt19937 to set a random number between -1 and 1.
        randAngles[k] = rnd_dis(mt_rand);
        // Adjust the range of the random values from [-1, 1] to [ub, lb].
        if (randAngles[k] < 0) { randAngles[k] = -1*lb[k]*randAngles[k]; }
        else { randAngles[k] = ub[k]*randAngles[k]; }
        // Check if joint k is a revolute unbounded (continuous) joint (This joint types come from Pinocchio)
        if (rob_model.joint_types[k].compare("JointModelRUBZ") == 0 || rob_model.joint_types[k].compare("JointModelRUBY") == 0 || rob_model.joint_types[k].compare("JointModelRUBX") == 0)
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
      randConfig = Eigen::VectorXd::Zero(rob_model.n_q, 1);

      for (Eigen::DenseIndex k = 0; k < rob_model.n_q; ++k)
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
  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model, std::vector<double> lower_bounds_v, std::vector<double> upper_bounds_v)
  {
    // QUESTION Should these lower and upper bounds be compared with rob_model.joint_pos_lb and rob_model.joint_pos_ub?

    // Assert that both the lower_bounds and upper_bounds vectors are of length equal to n_dof.
    custom_assert(lower_bounds_v.size() == rob_model.n_dof && upper_bounds_v.size() == rob_model.n_dof, "Error in " + std::string(__FUNCTION__) + "(): Lower and upper bound vectors must be of length equal to n_dof.");

    // double* ptr_lb = &lower_bounds_v[0];
    // double* ptr_ub = &upper_bounds_v[0];

    Eigen::Map<Eigen::VectorXd> lb(&lower_bounds_v[0], rob_model.n_dof);
    Eigen::Map<Eigen::VectorXd> ub(&upper_bounds_v[0], rob_model.n_dof);

    // std::cout << "lb: " << lb.transpose() << "\t ub: " << ub.transpose() << std::endl;

    return randomConfiguration(rob_model, lb, ub);
  }

  void print_model_data(Serial_Robot rob_info)
  {
      std::cout << "\n----- Robot model information: " << std::endl;
      print_indent("Model name = ",                         rob_info.name,               38);
      print_indent("Size of configuration vector = ",       rob_info.n_q,                38);
      print_indent("Number of joints (with universe) = ",   rob_info.n_joints,           38);
      print_indent("Number of DoF = ",                      rob_info.n_dof,              38);
      print_indent("Number of bodies = ",                   rob_info.n_bodies,           38);
      print_indent("Number of operational frames = ",       rob_info.n_frames,           38);
      print_indent("Gravity = ",                            rob_info.gravity,            38);
      print_indent("Joint torque bounds = ",                rob_info.joint_torque_limit, 38);
      print_indent("Joint configuration upper bounds = ",   rob_info.joint_pos_ub,       38);
      print_indent("Joint configuration lower bounds = ",   rob_info.joint_pos_lb,       38);
      print_indent("Joint velocity bounds = ",              rob_info.joint_vel_limit,    38);
      std::cout << std::endl;
      // std::cout << "\n----- Placement of each joint in the model: " << std::endl;
      // std::cout << "\n-----Name of each joint in the model: " << std::endl;
      // for (int k=0 ; k<rob_info.n_joints ; ++k)
      // {
      //     std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << rob_info.joint_names[k] << std::setw(10) << std::endl; // << data.oMi[k].translation().transpose()
      // }

      //
      // std::cout << "\n----- Placement of each frame in the model: " << std::endl;
      // for (int k=0 ; k<rob_info.n_frames ; ++k)
      // {
      //     std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << model.frames[k].name << std::setw(10) << data.oMf[k].translation().transpose() << std::endl;
      // }
  }

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
