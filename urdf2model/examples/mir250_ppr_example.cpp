#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    string ws_path = "/home/mtplnr/mpc_ws/urdf2casadi";
  // Example with MMO-500 URDF.

  // ---------------------------------------------------------------------
  // Create a model based on a URDF file
  // ---------------------------------------------------------------------
  std::string urdf_filename = ws_path+"/urdf2model/models/mir250/mir250_ppr.urdf";
  // Instantiate a Serial_Robot object called robot_model
  mecali::Serial_Robot robot_model;
  // Define (optinal) gravity vector to be used
  Eigen::Vector3d gravity_vector(0, 0, -9.81);
  // Eigen::Vector3d gravity_vector(0, 0, 0);
  // Create the model based on a URDF file
  robot_model.import_model(urdf_filename, gravity_vector);
  // robot_model.import_floating_base_model(urdf_filename, gravity_vector, true, true);
  //robot_model.import_planar_base_model(urdf_filename, gravity_vector, true, true);
  // For a floating base robot:
  // q = [global_base_position, global_base_quaternion, joint_positions]
  // v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
  // See: https://github.com/stack-of-tasks/pinocchio/issues/1137

  // Print some information related to the imported model (boundaries, frames, DoF, etc)
  robot_model.print_model_data();

  // ---------------------------------------------------------------------
  // Set functions for robot dynamics and kinematics
  // ---------------------------------------------------------------------
  // Set function for forward dynamics
  casadi::Function fd = robot_model.forward_dynamics();
  // // Set function for inverse dynamics
  casadi::Function id = robot_model.inverse_dynamics();
  casadi::Function G = robot_model.generalized_gravity();
  // // Set function for forward kinematics
  std::vector<std::string> required_Frames = {"lin_x_joint", "lin_y_joint", "rot_z_joint", "endeffector"};

  std::string end_effector_name = "endeffector";

  casadi::Function fkpos_ee = robot_model.forward_kinematics("position", end_effector_name);
  casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", end_effector_name);
  casadi::Function fk_ee = robot_model.forward_kinematics("transformation", end_effector_name);
  casadi::Function fk = robot_model.forward_kinematics("transformation",required_Frames);

  casadi::Function J_fd = robot_model.forward_dynamics_derivatives("jacobian");
  casadi::Function J_id = robot_model.inverse_dynamics_derivatives("jacobian");

  // casadi::Function fk       = robot_model.forward_kinematics("transformation", required_Frames);

  //casadi::Function fk_ee_pos = robot_model.forward_kinematics("position", end_effector_name);

  // ---------------------------------------------------------------------
  // Test the function - Floating base model
  // ---------------------------------------------------------------------
  // std::vector<double> global_base_position = {0.5, 1, 1.5};
  // std::vector<double> global_base_quaternion = {0, 0, 0, 1}; // Identity quaternion
  // std::vector<double> joint_positions = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0};

  // std::vector<double> q_vec;
  // q_vec.insert(q_vec.end(), global_base_position.begin(), global_base_position.end());
  // q_vec.insert(q_vec.end(), global_base_quaternion.begin(), global_base_quaternion.end());
  // q_vec.insert(q_vec.end(), joint_positions.begin(), joint_positions.end());
  // // q = [global_base_position, global_base_quaternion, joint_positions]

  // // Evaluate the function with a casadi::DMVector containing q_vec as input
  // casadi::DM T_res = fk_ee(casadi::DMVector{q_vec})[0];
  // std::cout << "Function result with q_vec input        : " << T_res << std::endl;

  // ---------------------------------------------------------------------
  // Test the function - Planar base model
  // ---------------------------------------------------------------------
  //std::vector<double> global_base_position = {0, 0};
  //std::vector<double> heading_angle_cos_sin = {1, 0};
  //std::vector<double> joint_positions = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0};

  //std::vector<double> q_vec;
  //q_vec.insert(q_vec.end(), global_base_position.begin(), global_base_position.end());
  //q_vec.insert(q_vec.end(), heading_angle_cos_sin.begin(), heading_angle_cos_sin.end());
  //q_vec.insert(q_vec.end(), joint_positions.begin(), joint_positions.end());
  // q = [global_base_position, heading_angle_cos_sin, joint_positions]

  // Evaluate the function with a casadi::DMVector containing q_vec as input
  //casadi::DM T_res = fk_ee(casadi::DMVector{q_vec})[0];
  //std::cout << "Function result with q_vec input        : " << T_res << std::endl;

  // ---------------------------------------------------------------------
  // Generate (or save) a function
  // ---------------------------------------------------------------------
  // Code-generate or save a function
  // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
  mecali::Dictionary codegen_options;
  codegen_options["c"] = false;
  codegen_options["save"] = true;
  mecali::generate_code(fd, "mir250_ppr_fd", codegen_options);
  mecali::generate_code(id, "mir250_ppr_id", codegen_options);
  mecali::generate_code(G, "mir250_ppr_G", codegen_options);
  //mecali::generate_code(fk_ee_pos, "mmo500_ppr_fk_ee_pos", codegen_options);
   mecali::generate_code(fkrot_ee, "mir250_ppr_fkrot_ee", codegen_options);
  mecali::generate_code(fk_ee, "mir250_ppr_fk_ee", codegen_options);
  mecali::generate_code(fk, "mir250_ppr_fk", codegen_options);
  mecali::generate_code(J_fd, "mir250_ppr_J_fd", codegen_options);
  mecali::generate_code(J_id, "mir250_ppr_J_id", codegen_options);

  robot_model.generate_json("mir250_ppr.json");

  // std::cout << fd << std::endl;
}
