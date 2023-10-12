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
  std::string urdf_filename = ws_path+"/urdf2model/models/gen3/GEN3-LITE.urdf";
  // Instantiate a Serial_Robot object called robot_model
  mecali::Serial_Robot robot_model;
  // Define (optinal) gravity vector to be used
  Eigen::Vector3d gravity_vector(0, 0, -9.81);
  // Create the model based on a URDF file
  robot_model.import_model(urdf_filename, gravity_vector);

  // Print some information related to the imported model (boundaries, frames, DoF, etc)
  robot_model.print_model_data();

  // ---------------------------------------------------------------------
  // Set functions for robot dynamics and kinematics
  // ---------------------------------------------------------------------
  // Set function for forward dynamics
  casadi::Function fd = robot_model.forward_dynamics();
  // // Set function for inverse dynamics
  casadi::Function id = robot_model.inverse_dynamics();
  casadi::Function M = robot_model.mass_matrix();
  casadi::Function Minv = robot_model.mass_inverse_matrix();
  casadi::Function C = robot_model.coriolis_matrix();
  casadi::Function G = robot_model.generalized_gravity();
  
  // // Set function for forward kinematics
  std::vector<std::string> required_Frames = {"J0", "J1", "J2", "J3", "J4", "J5", "TCP"};

  std::string end_effector_name = "TCP";
  casadi::Function fkpos_ee = robot_model.forward_kinematics("position", end_effector_name);
  casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", end_effector_name);
  casadi::Function fk_ee = robot_model.forward_kinematics("transformation", end_effector_name);
  casadi::Function fk = robot_model.forward_kinematics("transformation",required_Frames);

  casadi::Function J_fd = robot_model.forward_dynamics_derivatives("jacobian");
  casadi::Function J_id = robot_model.inverse_dynamics_derivatives("jacobian");

  // ---------------------------------------------------------------------
  // Generate (or save) a function
  // ---------------------------------------------------------------------
  // Code-generate or save a function
  // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
  mecali::Dictionary codegen_options;
  codegen_options["c"] = false;
  codegen_options["save"] = true;
  mecali::generate_code(fd, "gen3_lite_fd", codegen_options);
  mecali::generate_code(id, "gen3_lite_id", codegen_options);
  mecali::generate_code(M, "gen3_lite_M", codegen_options);
  mecali::generate_code(Minv, "gen3_lite_Minv", codegen_options);
  mecali::generate_code(C, "gen3_lite_C", codegen_options);
  mecali::generate_code(G, "gen3_lite_G", codegen_options);
  //mecali::generate_code(fk_ee_pos, "mmo500_ppr_fk_ee_pos", codegen_options);
   mecali::generate_code(fkrot_ee, "gen3_lite_fkrot_ee", codegen_options);
  mecali::generate_code(fk_ee, "gen3_lite_fk_ee", codegen_options);
  mecali::generate_code(fk, "gen3_lite_fk", codegen_options);
  mecali::generate_code(J_fd, "gen3_lite_J_fd", codegen_options);
  mecali::generate_code(J_id, "gen3_lite_J_id", codegen_options);

  robot_model.generate_json("gen3_lite.json");
}
