#include <casadi/casadi.hpp>
#include "model_interface.hpp"
#include "utils/debug_functions.hpp"

// NOTE: With the following code, you can look for any file containint "text" in ../ : grep -inr "text" ../

int main(int argc, char ** argv)
{
    // Example with robot urdf passed as argument, or Kinova Gen3 by default.
      std::string urdf_filename = (argc<=1) ? "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf" : argv[1];
      // std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf";
        // ../urdf2model/models/kortex_description/urdf/JACO3_URDF_V10rev.urdf
        // ../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf
        // ../urdf2model/models/iiwa_description/urdf/iiwa14.urdf
        // ../urdf2model/models/abb_common/urdf/irb120.urdf

      mecali::Serial_Robot robot_model;
      robot_model.import_model(urdf_filename);

      std::cout << "robot_model name: " << robot_model.name << std::endl;


      // #ifdef DEBUG
      robot_model.print_model_data();
      // #endif

      // Set functions
        casadi::Function aba    = robot_model.forward_dynamics();
        casadi::Function rnea   = robot_model.inverse_dynamics();
        // casadi::Function fk_pos = robot_model.forward_kinematics("position","EndEffector_Link");
        casadi::Function fk_pos = robot_model.forward_kinematics("position",robot_model.n_frames-1);
        casadi::Function fk_rot = robot_model.forward_kinematics("rotation",robot_model.n_frames-1);
        casadi::Function fk_T   = robot_model.forward_kinematics("transformation");

        // casadi::Function fk_pos_multiFrames_byString  = robot_model.forward_kinematics("position",std::vector<std::string>{"EndEffector_Link", "Actuator5", "Actuator2"});
        // casadi::Function fk_pos_multiFrames_byInt     = robot_model.forward_kinematics("position",std::vector<int>{18, 11, 5});

        casadi::Function fk_pos_allframes = robot_model.forward_kinematics("position");

        std::cout << "Position all frames: \n" << fk_pos_allframes << std::endl;

      // Test functions
        std::vector<double> q_vec((size_t)robot_model.n_q);
        Eigen::Map<mecali::ConfigVector>( q_vec.data(), robot_model.n_q, 1 ) = robot_model.neutral_configuration; // Populate q_vec with the robot's neutral configuration
        // std::vector<double> q_vec = {0.86602540378, 0.5, 0, 1, 0, -0.45, 1, 0, 0, 1, 0};
        std::vector<double> v_vec((size_t)robot_model.n_dof);
        Eigen::Map<mecali::TangentVector>(v_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
        // std::vector<double> v_vec = {0, 0, 0, 0, 0, 0, 0};
        std::vector<double> a_vec((size_t)robot_model.n_dof);
        Eigen::Map<mecali::TangentVector>(a_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
        // std::vector<double> a_vec = {0, 0, 0, 0, 0, 0, 0};
        std::vector<double> tau_vec((size_t)robot_model.n_dof);
        Eigen::Map<mecali::TangentVector>(tau_vec.data(),robot_model.n_dof,1) = Eigen::VectorXd::Zero(robot_model.n_dof);
        // std::vector<double> tau_vec = {0, 0, 0, 0, 0, 0, 0};

        casadi::DM ddq_res = aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
        casadi::DM tau_res = rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];
        casadi::DM pos_res = fk_pos(casadi::DMVector {q_vec})[0];
        casadi::DM rot_res = fk_rot(casadi::DMVector {q_vec})[0];

        std::cout << "ddq: " << ddq_res << std::endl;
        std::cout << "tau: " << tau_res << std::endl;
        std::cout << "EE_pos: " << pos_res << std::endl;
        std::cout << "EE_rot: " << rot_res << std::endl;
        std::cout << std::endl;

      // Generate random configurations
        mecali::print_indent("Neutral configuration = ",            robot_model.neutral_configuration, 38);
        mecali::print_indent("Random configuration = ",             robot_model.randomConfiguration(),  38);
        mecali::print_indent("Random config. w/ custom bounds = ",  robot_model.randomConfiguration(-0.94159*Eigen::VectorXd::Ones(robot_model.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model.n_dof)), 38);

      // mecali::Dictionary opts1;
      // opts1["c"]=false;
      // opts1["save"]=true;
      // mecali::generate_code(aba,"kin3_aba",opts1);
      // mecali::generate_code(rnea,"kin3_rnea",opts1);
      // mecali::generate_code(fk_pos,"kin3_fk_pos",opts1);
      // mecali::generate_code(fk_rot,"kin3_fk_rot",opts1);



    // // Example with another robot (ABB irb120)
    //   mecali::Serial_Robot robot_model_abb;
    //   robot_model_abb.import_model("../urdf2model/models/abb_common/urdf/irb120.urdf");
    //
    //   robot_model_abb.print_model_data();
    //
    //   mecali::print_indent("Neutral configuration = ",            robot_model_abb.neutral_configuration, 38);
    //   mecali::print_indent("Random configuration = ",             robot_model_abb.randomConfiguration(),  38);
    //   mecali::print_indent("Random config. w/ custom bounds = ",  robot_model_abb.randomConfiguration(-0.94159*Eigen::VectorXd::Ones(robot_model_abb.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model_abb.n_dof)),       38);
    //   mecali::print_indent("Random config. w/ vector bounds = ",  robot_model_abb.randomConfiguration(std::vector<double>{-2, -2.2, -3.0, -2.4, -2.5, -2.6}, std::vector<double>{2.1, 2.2, 2.3, 2.4, 2.5, 2.6}), 38);
    //
    //   casadi::Function irb120_forward_dynamics = robot_model_abb.aba;
    //   std::cout << "irb120 forward dynamics function: " << irb120_forward_dynamics << std::endl;
    //
    //   // mecali::Dictionary opts;
    //   // opts["c"]=true;
    //   // opts["save"]=true;
    //   // mecali::generate_code(irb120_forward_dynamics,"abb_fd_ext",opts);
    //   //
    //   // std::cout << "irb120 forward dynamics function loaded: " << casadi::Function::load("abb_fd_ext.casadi") << std::endl;
    //   std::cout << "#########################################################" << std::endl;
    //   mecali::Serial_Robot robot_model_abb_2;
    //   robot_model_abb_2.import_model("../urdf2model/models/abb_common/urdf/irb120.urdf",true);


}
