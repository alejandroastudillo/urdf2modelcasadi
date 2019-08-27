#include <casadi/casadi.hpp>
#include "model_interface.hpp"
#include "utils/debug_functions.hpp"

/*  NOTE: With the following code, you can look for any file containint "text" in ../ : grep -inr "text" ../
    TODO: Create more examples (let this be a really simple one)
*/
int main(int argc, char ** argv)
{
    // Example with Kinova Gen3 URDF.
      std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf";

    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
    // Create the model based on a URDF file
      robot_model.import_model(urdf_filename);

    // Print some information related to the imported model (boundaries, frames, DoF, etc)
      robot_model.print_model_data();

    // Set function for forward dynamics
      casadi::Function fwd_dynamics = robot_model.forward_dynamics();
    // Set function for inverse dynamics
      casadi::Function inv_dynamics = robot_model.inverse_dynamics();
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

    // Test a function with numerical values
      // Create a std::vector<double> of size robot_model.n_q (take care in case there are continuous joints in your model)
      std::vector<double> q_vec = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0, 1, 0};
      // Evaluate the function with a casadi::DMVector containing q_vec as input
      casadi::DM pos_res = fk_pos_oneframe_by_name(casadi::DMVector {q_vec})[0];
      std::cout << "Function result with q_vec input        : " << pos_res << std::endl;

      // You can also use robot's neutral configuration or any random configuration as input
      std::vector<double> q_vec_neutral((size_t)robot_model.n_q);
      Eigen::Map<mecali::ConfigVector>( q_vec_neutral.data(), robot_model.n_q, 1 ) = robot_model.neutral_configuration; // Populate q_vec_neutral with the robot's neutral configuration

      std::vector<double> q_vec_random((size_t)robot_model.n_q);
      Eigen::Map<mecali::ConfigVector>( q_vec_random.data(), robot_model.n_q, 1 ) = robot_model.randomConfiguration(); // Populate q_vec_neutral with a random configuration

      casadi::DM pos_neutral = fk_pos_oneframe_by_name(casadi::DMVector {q_vec_neutral})[0];
      casadi::DM pos_random  = fk_pos_oneframe_by_name(casadi::DMVector {q_vec_random})[0];

      std::cout << "Function result with q_vec_neutral input: " << pos_neutral << std::endl;
      std::cout << "Function result with q_vec_random input : " << pos_random  << std::endl;

    // Code-generate or save a function
      // If not setting options, function fk_T_1 (or any function) will only be c-code-generated as "first_function.c" (or any other name you set)
      mecali::generate_code(fk_T_1, "first_function");
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "my_function_T.casadi" (which can be loaded afterwards using casadi::Function::load("my_function_T.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fk_T_multiframes_by_name, "second_function", codegen_options);

}
