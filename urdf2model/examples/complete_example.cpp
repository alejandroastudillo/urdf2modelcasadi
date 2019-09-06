#include <casadi/casadi.hpp>
#include "model_interface.hpp"
#include "utils/debug_functions.hpp"

/*  NOTE: With the following code, you can look for any file containint "text" in ../ : grep -inr "text" ../
    TODO: Create more examples (let this be a really simple one)
*/
int main(int argc, char ** argv)
{
    // Example with robot urdf passed as argument, or Kinova Gen3 URDF by default.
      std::string urdf_filename = (argc<=1) ? "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf" : argv[1];
    /* Other URDF examples included in the models directory are the following
      "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V10rev.urdf"
      "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf"
      "../urdf2model/models/iiwa_description/urdf/iiwa14.urdf"
      "../urdf2model/models/abb_common/urdf/irb120.urdf"
      "../urdf2model/models/yumi/urdf/yumi.urdf"
    */

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

        // casadi::Function fk_pos_multiFrames_byString  = robot_model.forward_kinematics("transformation",std::vector<std::string>{"EndEffector_Link", "Actuator5", "Actuator2"});
        // casadi::Function fk_pos_multiFrames_byInt     = robot_model.forward_kinematics("position",std::vector<int>{18, 11, 5});

        casadi::Function fk_pos_allframes = robot_model.forward_kinematics("position");
        casadi::Function fk_rot_allframes = robot_model.forward_kinematics("rotation");
        casadi::Function fk_T_allframes   = robot_model.forward_kinematics("transformation");
        // std::vector<std::string> required_Frames = {"yumi_body",
        //     "yumi_link_1_l", "yumi_link_2_l", "yumi_link_3_l", "yumi_link_4_l", "yumi_link_5_l", "yumi_link_6_l", "yumi_link_7_l",
        //     "gripper_l_base", "gripper_l_finger_r", "gripper_l_finger_l",
        //     "yumi_link_1_r", "yumi_link_2_r", "yumi_link_3_r", "yumi_link_4_r", "yumi_link_5_r", "yumi_link_6_r", "yumi_link_7_r",
        //     "gripper_r_base", "gripper_r_finger_r", "gripper_r_finger_l" };
        //
        // casadi::Function fk_pos_allframes = robot_model.forward_kinematics("position", required_Frames);
        // casadi::Function fk_rot_allframes = robot_model.forward_kinematics("rotation", required_Frames);
        // casadi::Function fk_T_allframes   = robot_model.forward_kinematics("transformation", required_Frames);

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








}
