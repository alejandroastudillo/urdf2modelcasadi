#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // Example with Kinova Gen3 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/kortex_2dof/urdf/GEN3_URDF_V12.urdf";
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

      casadi::Function gravity = robot_model.generalized_gravity();

      casadi::Function coriolis = robot_model.coriolis_matrix();

      casadi::Function mass = robot_model.mass_inverse_matrix();

      // Test functions
      std::vector<double> q_vec((size_t)robot_model.n_q);
      Eigen::Map<mecali::ConfigVector>( q_vec.data(), robot_model.n_q, 1 ) = robot_model.neutral_configuration; // Populate q_vec with the robot's neutral configuration
      // std::vector<double> q_vec = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0, 1, 0};
      std::vector<double> v_vec((size_t)robot_model.n_dof);
      Eigen::Map<mecali::TangentVector>(v_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
      // std::vector<double> v_vec = {0, 0, 0, 0, 0, 0, 0};
      std::vector<double> a_vec((size_t)robot_model.n_dof);
      Eigen::Map<mecali::TangentVector>(a_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);


        casadi::DM tau_res = inv_dynamics(casadi::DMVector {q_vec, v_vec, a_vec})[0];
        casadi::DM g_res = gravity(casadi::DMVector {q_vec})[0];
        casadi::DM Minv_res = mass(casadi::DMVector {q_vec})[0];
        casadi::DM cor_res = coriolis(casadi::DMVector {q_vec, v_vec})[0];


        std::cout << "Tau: " << tau_res << std::endl;
        std::cout << "g: " << g_res << std::endl;
        std::cout << "Minv: " << Minv_res << std::endl;
        std::cout << "cor: " << cor_res << std::endl;


        // CasadiScalar        q_sx = casadi::SX::sym("q", robot_model.n_q);
        // CasadiScalar        v_sx = casadi::SX::sym("v", robot_model.n_dof);
        // CasadiScalar        a_sx = casadi::SX::sym("a", robot_model.n_dof);
        //
        // CasadiScalar        taures_sx(robot_model.n_dof, 1);
        // taures_sx = 1*a_sx;
        //
        // casadi::Function    inv_full ("inv_full", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {taures_sx});
    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      // mecali::Dictionary codegen_options;
      // codegen_options["c"]=false;
      // codegen_options["save"]=true;
      // mecali::generate_code(fwd_dynamics, "kin3_fd", codegen_options);
      // mecali::generate_code(inv_dynamics, "kin3_id", codegen_options);


}
