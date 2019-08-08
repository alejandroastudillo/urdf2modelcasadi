#include <casadi/casadi.hpp>
#include "model_interface.hpp"
#include "utils/debug_functions.hpp"

// TODO: Remove the "using namespace ...". It is better to explicitely put the namespace before each attribute like: mecali::Serial_Robot
// NOTE: With the following code, you can look for any file containint "text" in ../ : grep -inr "text" ../

using namespace mecali;

int main(int argc, char ** argv)
{
    // Example with robot urdf passed as argument, or Kinova Gen3 by default.
      std::string urdf_filename = (argc<=1) ? "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf" : argv[1];
        // ../urdf2model/models/kortex_description/urdf/JACO3_URDF_V10rev.urdf
        // ../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf
        // ../urdf2model/models/iiwa_description/urdf/iiwa14.urdf
        // ../urdf2model/models/abb_common/urdf/irb120.urdf

      Serial_Robot robot_model;
      robot_model = generate_model(urdf_filename);

      std::cout << "robot_model name: " << robot_model.name << std::endl;
      std::cout << "robot_model ABA: " << robot_model.aba << std::endl;

      std::vector<double> q_vec((size_t)robot_model.n_q);
      Eigen::Map<ConfigVector>( q_vec.data(), robot_model.n_q, 1 ) = robot_model.neutral_configuration; // Populate q_vec with the robot's neutral configuration
      // std::vector<double> q_vec = {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0};
      std::vector<double> v_vec((size_t)robot_model.n_dof);
      Eigen::Map<TangentVector>(v_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
      // std::vector<double> v_vec = {0, 0, 0, 0, 0, 0, 0};
      std::vector<double> a_vec((size_t)robot_model.n_dof);
      Eigen::Map<TangentVector>(a_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
      // std::vector<double> a_vec = {0, 0, 0, 0, 0, 0, 0};
      std::vector<double> tau_vec((size_t)robot_model.n_dof);
      Eigen::Map<TangentVector>(tau_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
      // std::vector<double> tau_vec = {0, 0, 0, 0, 0, 0, 0};

      casadi::DM ddq_res = robot_model.aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      casadi::DM tau_res = robot_model.rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];
      casadi::DM pos_res = robot_model.fk_pos(casadi::DMVector {q_vec})[0];
      casadi::DM rot_res = robot_model.fk_rot(casadi::DMVector {q_vec})[0];

      std::cout << "ddq: " << ddq_res << std::endl;
      std::cout << "tau: " << tau_res << std::endl;
      std::cout << "EE_pos: " << pos_res << std::endl;
      std::cout << "EE_rot: " << rot_res << std::endl;
      std::cout << std::endl;

      // #ifdef DEBUG
      print_model_data(robot_model);
      // #endif
      print_indent("Neutral configuration = ",            robot_model.neutral_configuration, 38);
      print_indent("Random configuration = ",             randomConfiguration(robot_model),  38);
      print_indent("Random config. w/ custom bounds = ",  randomConfiguration(robot_model, -0.94159*Eigen::VectorXd::Ones(robot_model.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model.n_dof)),       38);

      Dictionary opts1;
      opts1["c"]=true;
      opts1["save"]=true;
      generate_code(robot_model.aba,"kin3_aba",opts1);
      generate_code(robot_model.rnea,"kin3_rnea",opts1);
      generate_code(robot_model.fk_pos,"kin3_fk_pos",opts1);
      generate_code(robot_model.fk_rot,"kin3_fk_rot",opts1);


    // Example with another robot (ABB irb120)
      Serial_Robot robot_model_abb;
      robot_model_abb = generate_model("../urdf2model/models/abb_common/urdf/irb120.urdf");

      print_model_data(robot_model_abb);

      print_indent("Neutral configuration = ",            robot_model_abb.neutral_configuration, 38);
      print_indent("Random configuration = ",             randomConfiguration(robot_model_abb),  38);
      print_indent("Random config. w/ custom bounds = ",  randomConfiguration(robot_model_abb, -0.94159*Eigen::VectorXd::Ones(robot_model_abb.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model_abb.n_dof)),       38);
      print_indent("Random config. w/ vector bounds = ",  randomConfiguration(robot_model_abb, std::vector<double>{-2, -2.2, -3.0, -2.4, -2.5, -2.6}, std::vector<double>{2.1, 2.2, 2.3, 2.4, 2.5, 2.6}), 38);

      casadi::Function irb120_forward_dynamics = robot_model_abb.aba;
      std::cout << "irb120 forward dynamics function: " << irb120_forward_dynamics << std::endl;

      Dictionary opts;
      opts["c"]=true;
      opts["save"]=true;
      generate_code(irb120_forward_dynamics,"abb_fd_ext",opts);

      std::cout << "irb120 forward dynamics function loaded: " << casadi::Function::load("abb_fd_ext.casadi") << std::endl;

}
