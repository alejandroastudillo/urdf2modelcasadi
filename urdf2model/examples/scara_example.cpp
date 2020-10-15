#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // Example with Kinova Gen3 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/scara/urdf/EpsonG3_251.urdf";
    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
    // Define (optinal) gravity vector to be used
      Eigen::Vector3d gravity_vector(0,0,-9.81);
    // Create the model based on a URDF file
      robot_model.import_model(urdf_filename, gravity_vector);

    // Print some information related to the imported model (boundaries, frames, DoF, barycentric parameters, etc)
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
      casadi::Function fk_T = robot_model.forward_kinematics();

      // casadi::Function expressions = robot_model.robot_expressions(std::vector<std::string>{"EndEffector_Link"}, true);


    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      // mecali::Dictionary codegen_options;
      // codegen_options["c"]=false;
      // codegen_options["save"]=true;
      // mecali::generate_code(fwd_dynamics, "scara_fd", codegen_options);
      // mecali::generate_code(inv_dynamics, "scara_id", codegen_options);
      //
      // mecali::generate_code(fk_T, "scara_fk", codegen_options);
      //
      // mecali::generate_code(regressor, "scara_regressor", codegen_options);
      //
      // mecali::generate_code(mass_inverse, "scara_mass_inverse", codegen_options);
      // mecali::generate_code(coriolis, "scara_coriolis", codegen_options);
      // mecali::generate_code(gravity, "scara_gravity", codegen_options);

    // ---------------------------------------------------------------------
    // Print functions
    // ---------------------------------------------------------------------
      std::cout << "Function scada_fd: " << fwd_dynamics << std::endl;
      std::cout << "Function scada_id: " << inv_dynamics << std::endl;
      std::cout << "Function scada_regressor: " << regressor << std::endl;
      std::cout << "Function scada_fk: " << fk_T << std::endl;
      std::cout << "Function scada_mass_inverse: " << mass_inverse << std::endl;

      std::vector<double> q_vec = {0.86, 0.5, -0.1};
      std::vector<double> v_vec = {0.3, 0.02, 0.05};
      std::vector<double> tau_vec = {2.5, 5.6, 1.0};
      std::vector<double> ddq_vec = {-1230.59, 2497.78, -6.23602};


      casadi::DM fd_res = fwd_dynamics(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      casadi::DM id_res = inv_dynamics(casadi::DMVector {q_vec, v_vec, ddq_vec})[0];
      casadi::DM regressor_res = regressor(casadi::DMVector {q_vec, v_vec, ddq_vec})[0];
      casadi::DM mass_inverse_res = mass_inverse(casadi::DMVector {q_vec})[0];
      std::cout << "fd_res: " << fd_res << std::endl;
      std::cout << "id_res: " << id_res << std::endl;
      std::cout << "regressor_res: " << regressor_res << std::endl;
      std::cout << "mass_inverse_res: " << mass_inverse_res << std::endl;



}
