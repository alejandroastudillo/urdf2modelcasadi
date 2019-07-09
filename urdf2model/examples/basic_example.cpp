#include <casadi/casadi.hpp>
#include "../src/interface/pinocchio_interface.hpp"

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

    std::cout << "ddq: " << ddq_res << std::endl;

    casadi::DM tau_res = robot_model.rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];

    std::cout << "tau: " << tau_res << std::endl;

    casadi::DM pos_res = robot_model.fk_pos(casadi::DMVector {q_vec})[0];

    std::cout << "EE_pos: " << pos_res << std::endl;
    std::cout << std::endl;

    // #ifdef DEBUG
    print_model_data(robot_model);
    // #endif

    // std::cout << "neutral: " << robot_model.neutral_configuration.transpose() << std::endl;

    std::cout << "home configuration: " << robot_model.neutral_configuration.transpose() << std::endl << std::endl;

    std::cout << "random configuration: " << randomConfiguration(robot_model).transpose() << std::endl << std::endl;

    // std::cout << "random configuration extended: " << randomConfiguration(robot_model, -3.14159*Eigen::VectorXd::Ones(robot_model.n_q), 3.14159*Eigen::VectorXd::Ones(robot_model.n_q)).transpose() << std::endl;

    std::cout << "random configuration extended: " << randomConfiguration(robot_model, -3.14159*Eigen::VectorXd::Ones(robot_model.n_dof), 3.14159*Eigen::VectorXd::Ones(robot_model.n_dof)).transpose() << std::endl;

    //
    // Serial_Robot robot_model_2;
    // robot_model_2 = generate_model("../urdf2model/models/iiwa_description/urdf/iiwa14.urdf");
    //
    // print_model_data(robot_model_2);

    // robot_init(filename);
    // // execute_tests();
    //
    // test_casadi_aba();
    // test_casadi_rnea();
    // test_casadi_fk();
    //
    // #ifdef DEBUG
    //   print_model_data();
    // #endif

}
