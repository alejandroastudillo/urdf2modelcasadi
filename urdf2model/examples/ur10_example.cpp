#include <casadi/casadi.hpp>
#include "model_interface.hpp"

using namespace std;
int main()
{
  // Example with UR10 URDF.

  // ---------------------------------------------------------------------
  // Create a model based on a URDF file
  // ---------------------------------------------------------------------
  std::string urdf_filename = "../urdf2model/models/ur10/ur10e.urdf";
  // Instantiate a Serial_Robot object called robot_model
  mecali::Serial_Robot robot_model;
  // Create the model based on a URDF file
  robot_model.import_model(urdf_filename);

  // Print some information related to the imported model (boundaries, frames, DoF, etc)
  robot_model.print_model_data();

  // ---------------------------------------------------------------------
  // Set functions for robot dynamics and kinematics
  // ---------------------------------------------------------------------
  // Set function for forward dynamics
    casadi::Function fwd_dynamics = robot_model.forward_dynamics();
  // Set function for inverse dynamics
    casadi::Function inv_dynamics = robot_model.inverse_dynamics();
  // Set function for forward kinematics
  std::vector<std::string> required_Frames = {"shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"};

  casadi::Function fkpos_ee = robot_model.forward_kinematics("position", "tool0");
  casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", "tool0");
  casadi::Function fk_ee = robot_model.forward_kinematics("transformation", "tool0");
  casadi::Function fk = robot_model.forward_kinematics("transformation", required_Frames);
  casadi::Function fd = robot_model.forward_dynamics();
  casadi::Function id = robot_model.inverse_dynamics();
  casadi::Function J_fd = robot_model.forward_dynamics_derivatives("jacobian");
  casadi::Function J_id = robot_model.inverse_dynamics_derivatives("jacobian");

  robot_model.generate_json("ur10e.json");

  std::vector<double> q_vec = {0, -1.0472, 1.0472, -1.5708, -1.5708, 0};

  casadi::DM pos_res = fkpos_ee(casadi::DMVector {q_vec})[0];

  std::cout << "pos_ee 1: " << pos_res << std::endl;

  // ---------------------------------------------------------------------
  // Generate (or save) a function
  // ---------------------------------------------------------------------
  // Code-generate or save a function
  // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
  mecali::Dictionary codegen_options;
  codegen_options["c"] = false;
  codegen_options["save"] = true;
  // mecali::generate_code(fkpos_ee, "ur10_fkpos_ee", codegen_options);
  // mecali::generate_code(fkrot_ee, "ur10_fkrot_ee", codegen_options);
  mecali::generate_code(fk_ee, "ur10e_fk_ee", codegen_options);
  mecali::generate_code(fk, "ur10e_fk", codegen_options);
  mecali::generate_code(fd, "ur10e_fd", codegen_options);
  mecali::generate_code(id, "ur10e_id", codegen_options);
  mecali::generate_code(J_fd, "ur10e_J_fd", codegen_options);
  mecali::generate_code(J_id, "ur10e_J_id", codegen_options);
}
