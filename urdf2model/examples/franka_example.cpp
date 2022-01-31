#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // Example with Kinova Gen3 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/franka_panda/franka_with_gripper.urdf";
    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
    // Define (optinal) gravity vector to be used
      Eigen::Vector3d gravity_vector(0,0,-9.81);
    // Create the model based on a URDF file
      robot_model.import_model(urdf_filename, gravity_vector);


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
      casadi::Function gravity = robot_model.generalized_gravity();
      casadi::Function coriolis = robot_model.coriolis_matrix();
      casadi::Function mass_inverse = robot_model.mass_inverse_matrix();
    // Set function for joint torque regressor: regressor(q, dq, ddq)*barycentric_params = tau
      casadi::Function regressor = robot_model.joint_torque_regressor();
    // Set function for forward kinematics
      // Setting the first argument as "position" means that the function is going to output a 3x1 position vector for each frame.
      casadi::Function fk_pos = robot_model.forward_kinematics("position", "panda_hand");
      // Setting the first argument as "rotation" means that the function is going to output a 3x3 rotation matrix for each frame.
      casadi::Function fk_rot = robot_model.forward_kinematics("rotation", "panda_hand");

      casadi::Function fk = robot_model.forward_kinematics("transformation", "panda_hand");

      // casadi::Function expressions = robot_model.robot_expressions(std::vector<std::string>{"panda_hand"}, true);


    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fwd_dynamics, "panda_with_gripper_fd", codegen_options);
      mecali::generate_code(inv_dynamics, "panda_with_gripper_id", codegen_options);
      
      mecali::generate_code(fk, "panda_with_gripper_fk", codegen_options);
      mecali::generate_code(fk_pos, "panda_with_gripper_fkpos", codegen_options);
      mecali::generate_code(fk_rot, "panda_with_gripper_fkrot", codegen_options);
      
      mecali::generate_code(regressor, "panda_with_gripper_regressor", codegen_options);
      
      mecali::generate_code(mass_inverse, "panda_with_gripper_mass_inverse", codegen_options);
      mecali::generate_code(coriolis, "panda_with_gripper_coriolis", codegen_options);
      mecali::generate_code(gravity, "panda_with_gripper_gravity", codegen_options);

    //   mecali::generate_code(expressions, "kin3_expressions_noG", codegen_options);

    //   std::cout << "Function expressions: " << expressions << std::endl;
    //   std::vector<double> q_vec = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0, 1, 0};
    //   std::vector<double> v_vec = {0, 0, 0, 0, 0, 0, 0};
    //   std::vector<double> tau_vec = {0, 0, 0, 0, 0, 0, 0};
    //   std::vector<double> vp_vec = {0, 0};
    //   std::vector<double> wp_vec = {0};

    //   casadi::DM ode_aug_res = expressions(casadi::DMVector {q_vec, v_vec, tau_vec, vp_vec, wp_vec})[0];
    //   casadi::DM pos_res = expressions(casadi::DMVector {q_vec, v_vec, tau_vec, vp_vec, wp_vec})[1];
    //   std::cout << "Ode_aug_res: " << ode_aug_res << std::endl;
    //   std::cout << "pos_res: " << pos_res << std::endl;

    //   std::cout << "Instructions: " << expressions.n_instructions() << std::endl;

    //   // q_sx, v_sx, tau_sx, vp_sx, wp_sx

}
