#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // Example with UR10 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/yumi/urdf/yumi.urdf";
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
    std::vector<std::string> required_Frames = {"yumi_body",
        "yumi_link_1_l", "yumi_link_2_l", "yumi_link_3_l", "yumi_link_4_l", "yumi_link_5_l", "yumi_link_6_l", "yumi_link_7_l",
        "gripper_l_base", "gripper_l_finger_r", "gripper_l_finger_l",
        "yumi_link_1_r", "yumi_link_2_r", "yumi_link_3_r", "yumi_link_4_r", "yumi_link_5_r", "yumi_link_6_r", "yumi_link_7_r",
        "gripper_r_base", "gripper_r_finger_r", "gripper_r_finger_l" };

      casadi::Function fk_pos = robot_model.forward_kinematics("transformation", required_Frames);



    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fk_pos, "yumi_fk", codegen_options);



}
