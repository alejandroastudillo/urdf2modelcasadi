# INTERFACE - Rigid body dynamics from URDF into CasADi

### Build from Source
* Clone this repository.
* Go the the build directory.
* Open the `configure.sh` file and modify the paths of `CASADI_DIRECTORY`, `PINOCCHIO_INCLUDE`, and `EIGEN_INCLUDE` with those from your system.
* You can also set a custom INSTALL_FOLDER.
* Open a terminal inside the build directory.
* Execute `source configure.sh`.

### Dependencies
* CMake
```
sudo apt-get install cmake
```
* Boost for unit-testing and some asserts
```
sudo apt-get install libboost-all-dev
```
* URDFDOM for URDF parser
```
sudo apt-get install liburdfdom-dev
```

* CASADI >= 3.4.5 (with pkg-config support)

* Pinocchio (with Casadi interface - needs pkg-config support) from branch 'devel' - commit 0a8094da7feb9a5822a120acc4b6817c06722da7
```
# Clone Pinocchio's repository (devel branch)
git clone -b devel https://github.com/stack-of-tasks/pinocchio.git
# Checkout the commit I have been using
cd pinocchio
# Fetch pinocchio submodules
git submodule update --init --recursive
# Build from source
cd build
export CMAKE_PREFIX_PATH=/home/alejandro/phd_software/casadi_source/build:$CMAKE_PREFIX_PATH
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_CASADI_SUPPORT=ON
sudo make -j4
sudo make install
```
From Justin's fork:
```
# Clone Pinocchio's repository (devel branch)
git clone -b topic/casadi https://github.com/jcarpent/pinocchio.git
# Checkout the commit I have been using
cd pinocchio
git checkout 0981fc1b0dad8c303ab143b8aca2e61e9a450edb
# Build from source
cd /pinocchio/build
export CMAKE_PREFIX_PATH=/home/alejandro/phd_software/casadi_source/build:$CMAKE_PREFIX_PATH
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_CASADI_SUPPORT=ON
sudo make -j4
sudo make install

```
* C++11 (for randomConfiguration)


### Example
```cpp
#include <casadi/casadi.hpp>
#include "model_interface.hpp"

int main()
{
    // Example with Kinova Gen3 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf";
    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
    // Create the model based on a URDF file
      robot_model.import_model(urdf_filename);

    // ---------------------------------------------------------------------
    // Look inside the robot_model object. What variables can you fetch?
    // ---------------------------------------------------------------------
    // Get some variables contained in the robot_model object
      std::string name      = robot_model.name;
      int         n_q       = robot_model.n_q;
      int         n_joints  = robot_model.n_joints;
      int         n_dof     = robot_model.n_dof;
      int         n_frames  = robot_model.n_frames;
      std::vector<std::string> joint_names = robot_model.joint_names;
      std::vector<std::string> joint_types = robot_model.joint_types;
      Eigen::VectorXd          gravity     = robot_model.gravity;
      Eigen::VectorXd          joint_torque_limit    = robot_model.joint_torque_limit;
      Eigen::VectorXd          joint_pos_ub          = robot_model.joint_pos_ub;
      Eigen::VectorXd          joint_pos_lb          = robot_model.joint_pos_lb;
      Eigen::VectorXd          joint_vel_limit       = robot_model.joint_vel_limit;
      Eigen::VectorXd          neutral_configuration = robot_model.neutral_configuration;
    // Print some information related to the imported model (boundaries, frames, DoF, etc)
      robot_model.print_model_data();

    // ---------------------------------------------------------------------
    // Set functions for robot dynamics and kinematics
    // ---------------------------------------------------------------------
    // Set function for forward dynamics
      casadi::Function fwd_dynamics = robot_model.forward_dynamics();
    // Set function for inverse dynamics
      casadi::Function inv_dynamics = robot_model.inverse_dynamics();

    // Set functions for mass_inverse matrix, coriolis matrix, and generalized gravity vector
      casadi::Function gen_gravity = robot_model.generalized_gravity();

      casadi::Function coriolis = robot_model.coriolis_matrix();

      casadi::Function mass_inverse = robot_model.mass_inverse_matrix();

    // Set function for joint torque regressor: regressor(q, dq, ddq)*barycentric_params = tau
      casadi::Function regressor = robot_model.joint_torque_regressor();

    // Set function for forward kinematics
      // The forward kinematics function can be set in multiple ways
      // Calling forward_kinematics without any argument generates a function which outputs a transformation matrix for each frame in the robot.
      casadi::Function fk_T_1 = robot_model.forward_kinematics();
      // The first optional argument refers to the content of the output function: it can be set to be "position", "rotation", or "transformation"
      // Setting the first argument as "transformation" is just the same as not including any argument: outputs a 4x4 T matrix for each frame.
      casadi::Function fk_T_2 = robot_model.forward_kinematics("transformation");
      // Setting the first argument as "position" means that the function is going to output a 3x1 position vector for each frame.
      casadi::Function fk_pos = robot_model.forward_kinematics("position");
      // Setting the first argument as "rotation" means that the function is going to output a 3x3 rotation matrix for each frame.
      casadi::Function fk_rot = robot_model.forward_kinematics("rotation");

      // You can also generate a F.K. function for specific frames (using the frame name or index, which you can see after executing "robot_model.print_model_data()"" )
      casadi::Function fk_T_multiframes_by_name  = robot_model.forward_kinematics("transformation", std::vector<std::string>{"EndEffector_Link", "Actuator5", "Shoulder_Link"});
      casadi::Function fk_T_multiframes_by_index = robot_model.forward_kinematics("transformation", std::vector<int>{18, 11, 4});
      casadi::Function fk_pos_oneframe_by_name   = robot_model.forward_kinematics("position", "EndEffector_Link");
      casadi::Function fk_pos_oneframe_by_index  = robot_model.forward_kinematics("position", 18);

    // ---------------------------------------------------------------------
    // Generate random configuration vectors
    // ---------------------------------------------------------------------
      // You can generate a random configuration vector (size n_q) that takes into account upper and lower boundaries of the joint angles.
      Eigen::VectorXd random_conf_vector = robot_model.randomConfiguration();
      // You can also use your own upper and lower boundaries of the joint angles using Eigen::VectorXd of size n_dof,
      Eigen::VectorXd random_conf_vector_bounded = robot_model.randomConfiguration(-0.94159*Eigen::VectorXd::Ones(robot_model.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model.n_dof));
      // or using std::vector<double> of size n_dof.
      Eigen::VectorXd random_conf_vector_bounded2 = robot_model.randomConfiguration(std::vector<double>{-2, -2.2, -3.0, -2.4, -2.5, -2.6, -1}, std::vector<double>{2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 0.75});

      // mecali::print_indent("Random configuration (bounded) = ", random_conf_vector_bounded, 38);

    // ---------------------------------------------------------------------
    // Evaluate a kinematics or dynamics function
    // ---------------------------------------------------------------------
    // Test a function with numerical values
      /* Create a std::vector<double> of size robot_model.n_q (take care in case there are continuous joints in your model)

         For the Kinova Gen3 robot n_dof = 7, but n_q = 11, since {q1. q3. q5. q7} are continuous (unbounded) joints.
         Continuous joints are not represented just by q_i, but by [cos(q_i), sin(q_i)].
         The configuration vector is then set as: [cos(q1), sin(q1), q2, cos(q3), sin(q3), q4, cos(q5), sin(q5), q6, cos(q7), sin(q7)]
      */
      std::vector<double> q_vec = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0.2, 1, 0};
      // Evaluate the function with a casadi::DMVector containing q_vec as input
      casadi::DM pos_res = fk_pos_oneframe_by_name(casadi::DMVector {q_vec})[0];
      std::cout << "Function result with q_vec input        : " << pos_res << std::endl;

      // You can also use robot's neutral configuration as input
      std::vector<double> q_vec_neutral((size_t)robot_model.n_q);
      Eigen::Map<mecali::ConfigVector>( q_vec_neutral.data(), robot_model.n_q, 1 ) = robot_model.neutral_configuration; // Populate q_vec_neutral with the robot's neutral configuration

      casadi::DM pos_neutral = fk_pos_oneframe_by_name(casadi::DMVector {q_vec_neutral})[0];
      std::cout << "Function result with q_vec_neutral input: " << pos_neutral << std::endl;

      // or use a random configuration as input
      std::vector<double> q_vec_random((size_t)robot_model.n_q);
      Eigen::Map<mecali::ConfigVector>( q_vec_random.data(), robot_model.n_q, 1 ) = robot_model.randomConfiguration(); // Populate q_vec_neutral with a random configuration

      casadi::DM pos_random  = fk_pos_oneframe_by_name(casadi::DMVector {q_vec_random})[0];
      std::cout << "Function result with q_vec_random input : " << pos_random  << std::endl;

    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If not setting options, function fk_T_1 (or any function) will only be C-code-generated as "first_function.c" (or any other name you set)
      mecali::generate_code(fk_T_1, "first_function");
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fk_T_multiframes_by_name, "second_function", codegen_options);
}
```

## Citing

If you use this library, we would be grateful if you could cite the following paper: 

[Mixed Use of Analytical Derivatives and Algorithmic Differentiation for NMPC of Robot Manipulators](https://doi.org/10.1016%2Fj.ifacol.2021.11.156)
```
@inproceedings{Astudillo2021,
    doi = {10.1016/j.ifacol.2021.11.156},
    url = {https://doi.org/10.1016%2Fj.ifacol.2021.11.156},
    year = 2021,
    month = oct,
    publisher = {Elsevier {BV}},
    author = {Alejandro Astudillo and Justin Carpentier and Joris Gillis and Goele Pipeleers and Jan Swevers},
    title = {Mixed Use of Analytical Derivatives and Algorithmic Differentiation for {NMPC} of Robot Manipulators},
    booktitle = {Modeling, Estimation and Control Conference {MECC} 2021}  
}
```
