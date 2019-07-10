#include <casadi/casadi.hpp>
#include "../src/interface/pinocchio_interface.hpp"
#include "../src/utils/debug_functions.hpp"

using namespace casadi;

int main(int argc, char ** argv)
{
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

    std::cout << "ddq: " << ddq_res << std::endl;
    std::cout << "tau: " << tau_res << std::endl;
    std::cout << "EE_pos: " << pos_res << std::endl;
    std::cout << std::endl;

    // #ifdef DEBUG
    print_model_data(robot_model);
    // #endif
    print_indent("Neutral configuration = ",   robot_model.neutral_configuration,       38);
    print_indent("Random configuration = ",   randomConfiguration(robot_model),       38);
    print_indent("Random config. w/ custom bounds = ",   randomConfiguration(robot_model, -0.94159*Eigen::VectorXd::Ones(robot_model.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model.n_dof)),       38);



    // Serial_Robot robot_model_2;
    // robot_model_2 = generate_model("../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11.urdf");
    //
    // print_model_data(robot_model_2);
    //
    // std::cout << "Neutral configuration: " << robot_model_2.neutral_configuration.transpose() << std::endl;
    // std::cout << "Random configuration: " << randomConfiguration(robot_model_2).transpose() << std::endl;
    // std::cout << "Random configuration with custom bounds: " << randomConfiguration(robot_model_2, -0.94159*Eigen::VectorXd::Ones(robot_model_2.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model_2.n_dof)).transpose() << std::endl;
    //
    //
    // Serial_Robot robot_model_3;
    // robot_model_3 = generate_model("../urdf2model/models/iiwa_description/urdf/iiwa14.urdf");
    //
    // print_model_data(robot_model_3);
    //
    // std::cout << "Neutral configuration: " << robot_model_3.neutral_configuration.transpose() << std::endl;
    // std::cout << "Random configuration: " << randomConfiguration(robot_model_3).transpose() << std::endl;
    // std::cout << "Random configuration with custom bounds: " << randomConfiguration(robot_model_3, -0.94159*Eigen::VectorXd::Ones(robot_model_3.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model_3.n_dof)).transpose() << std::endl;
    //
    // Serial_Robot robot_model_4;
    // robot_model_4 = generate_model("../urdf2model/models/iiwa_description/urdf/iiwa14.urdf");
    //
    // print_model_data(robot_model_4);
    //
    // std::cout << "Neutral configuration: " << robot_model_4.neutral_configuration.transpose() << std::endl;
    // std::cout << "Random configuration: " << randomConfiguration(robot_model_4).transpose() << std::endl;
    // std::cout << "Random configuration with custom bounds: " << randomConfiguration(robot_model_4, -0.94159*Eigen::VectorXd::Ones(robot_model_4.n_dof), 0.94159*Eigen::VectorXd::Ones(robot_model_4.n_dof)).transpose() << std::endl;
    //

}
