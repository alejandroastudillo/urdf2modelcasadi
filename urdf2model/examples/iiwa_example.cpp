#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
  // Example with Kinova Gen3 URDF.

  // ---------------------------------------------------------------------
  // Create a model based on a URDF file
  // ---------------------------------------------------------------------
  std::string urdf_filename = "../urdf2model/models/iiwa_description/urdf/iiwa7.urdf";
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
  //   casadi::Function fwd_dynamics = robot_model.forward_dynamics();
  // // Set function for inverse dynamics
  //   casadi::Function inv_dynamics = robot_model.inverse_dynamics();
  // // Set function for forward kinematics
  std::vector<std::string> required_Frames = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7", "iiwa_joint_ee"};

  casadi::Function fkpos_ee = robot_model.forward_kinematics("position", "iiwa_joint_ee");
  casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", "iiwa_joint_ee");
  casadi::Function fk_ee = robot_model.forward_kinematics("transformation", "iiwa_joint_ee");
  casadi::Function fk = robot_model.forward_kinematics("transformation", required_Frames);
  casadi::Function fd = robot_model.forward_dynamics();
  casadi::Function id = robot_model.inverse_dynamics();
  casadi::Function J_fd = robot_model.forward_dynamics_derivatives("jacobian");
  casadi::Function J_id = robot_model.inverse_dynamics_derivatives("jacobian");

  robot_model.generate_json("iiwa7.json");

  // ---------------------------------------------------------------------
  // Generate (or save) a function
  // ---------------------------------------------------------------------
  // Code-generate or save a function
  // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
  mecali::Dictionary codegen_options;
  codegen_options["c"] = false;
  codegen_options["save"] = true;
  mecali::generate_code(fkpos_ee, "iiwa7_fkpos_ee", codegen_options);
  mecali::generate_code(fkrot_ee, "iiwa7_fkrot_ee", codegen_options);
  mecali::generate_code(fk_ee, "iiwa7_fk_ee", codegen_options);
  mecali::generate_code(fk, "iiwa7_fk", codegen_options);
  mecali::generate_code(fd, "iiwa7_fd", codegen_options);
  mecali::generate_code(id, "iiwa7_id", codegen_options);
  mecali::generate_code(J_fd, "iiwa7_J_fd", codegen_options);
  mecali::generate_code(J_id, "iiwa7_J_id", codegen_options);
}
