#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
  // Example with Kinova Gen3 URDF.

  // ---------------------------------------------------------------------
  // Create a model based on a URDF file
  // ---------------------------------------------------------------------
  std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/GEN3_URDF_V12_2f_85.urdf";
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
  std::vector<std::string> required_Frames = {"Actuator1", "Actuator2", "Actuator3", "Actuator4", "Actuator5", "Actuator6", "Actuator7", "EndEffector", "LEFT_TIP", "RIGHT_TIP"};

  casadi::Function fkpos_ee = robot_model.forward_kinematics("position", "EndEffector");
  casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", "EndEffector");
  casadi::Function fk_ee = robot_model.forward_kinematics("transformation", "EndEffector");
  casadi::Function fk = robot_model.forward_kinematics("transformation", required_Frames);
  casadi::Function fd = robot_model.forward_dynamics();
  casadi::Function id = robot_model.inverse_dynamics();

  casadi::Function J_fd = robot_model.forward_dynamics_derivatives("jacobian");
  casadi::Function J_id = robot_model.inverse_dynamics_derivatives("jacobian");

  robot_model.generate_json("kinova_gripper.json");

  // ---------------------------------------------------------------------
  // Generate (or save) a function
  // ---------------------------------------------------------------------
  // Code-generate or save a function
  // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
  mecali::Dictionary codegen_options;
  codegen_options["c"] = false;
  codegen_options["save"] = true;
  mecali::generate_code(fkpos_ee, "kinova_gripper_fkpos_ee", codegen_options);
  mecali::generate_code(fkrot_ee, "kinova_gripper_fkrot_ee", codegen_options);
  mecali::generate_code(fk_ee, "kinova_gripper_fk_ee", codegen_options);
  mecali::generate_code(fk, "kinova_gripper_fk", codegen_options);
  mecali::generate_code(fd, "kinova_gripper_fd", codegen_options);
  mecali::generate_code(id, "kinova_gripper_id", codegen_options);

  mecali::generate_code(J_fd, "kinova_gripper_J_fd", codegen_options);
  mecali::generate_code(J_id, "kinova_gripper_J_id", codegen_options);
}
