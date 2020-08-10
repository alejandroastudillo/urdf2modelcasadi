#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // Example with UR10 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/ur10/ur10_robot.urdf";
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
    //   casadi::Function fwd_dynamics = robot_model.forward_dynamics();
    // // Set function for inverse dynamics
    //   casadi::Function inv_dynamics = robot_model.inverse_dynamics();
    // // Set function for forward kinematics
      std::vector<std::string> required_Frames = {"shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "ee_link" };

      casadi::Function fkpos_ee = robot_model.forward_kinematics("position", "ee_link");
      casadi::Function fkrot_ee = robot_model.forward_kinematics("rotation", "ee_link");
      casadi::Function fk_ee    = robot_model.forward_kinematics("transformation", "ee_link");
      casadi::Function fk       = robot_model.forward_kinematics("transformation", required_Frames);
      casadi::Function fd       = robot_model.forward_dynamics();
      casadi::Function id       = robot_model.inverse_dynamics();

      robot_model.generate_json("ur10.json");



    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fkpos_ee, "ur10_fkpos_ee", codegen_options);
      mecali::generate_code(fkrot_ee, "ur10_fkrot_ee", codegen_options);
      mecali::generate_code(fk_ee, "ur10_fk_ee", codegen_options);
      mecali::generate_code(fk, "ur10_fk", codegen_options);
      mecali::generate_code(fd, "ur10_fd", codegen_options);
      mecali::generate_code(id, "ur10_id", codegen_options);

}
