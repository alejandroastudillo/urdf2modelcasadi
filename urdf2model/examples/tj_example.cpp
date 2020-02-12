#include <casadi/casadi.hpp>
#include "model_interface.hpp"

int main()
{
    // Example with Kinova Gen3 URDF.

    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename_1 = "../urdf2model/models/kortex_description/urdf/GEN3_URDF_V12.urdf";
      std::string urdf_filename_2 = "../urdf2model/models/kortex_2dof/urdf/GEN3_URDF_V12_w_base1.urdf";
      std::string urdf_filename_3 = "../urdf2model/models/kortex_2dof/urdf/GEN3_URDF_V12_wo_base.urdf";

    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model_7dof;
      mecali::Serial_Robot reduced_robot_model;
      mecali::Serial_Robot robot_model_2dof;
      mecali::Serial_Robot robot_model_2dof_wobase;
    // Define (optinal) gravity vector to be used
      Eigen::Vector3d gravity_vector(0,0,0);
    // Create models based on a URDF files
      robot_model_7dof.import_model(urdf_filename_1, gravity_vector);

      robot_model_2dof.import_model(urdf_filename_2, gravity_vector);

      robot_model_2dof_wobase.import_model(urdf_filename_3, gravity_vector);

      // Define list of joints to be locked (by name)
      std::vector<std::string> list_of_joints_to_lock_by_name = {"Actuator3","Actuator4","Actuator5","Actuator6","Actuator7"};
      // Define (optinal) robot configuration where joints should be locked
      std::vector<double> q_init_vec = {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0};
      Eigen::VectorXd q_init = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_init_vec.data(), q_init_vec.size());
      // Create the model based on a URDF file
      reduced_robot_model.import_reduced_model(urdf_filename_1, list_of_joints_to_lock_by_name, q_init, gravity_vector);

    // ---------------------------------------------------------------------
    // Compare kinematics and dynamics of the generated models
    // ---------------------------------------------------------------------
    casadi::Function fk_pos_1   = reduced_robot_model.forward_kinematics("transformation", "EndEffector_Link");
    casadi::Function fk_pos_2   = robot_model_2dof.forward_kinematics("transformation", "EndEffector_Link");
    casadi::Function fk_pos_3   = robot_model_2dof_wobase.forward_kinematics("transformation", "EndEffector_Link");

    casadi::Function fd_1   = reduced_robot_model.forward_dynamics();
    casadi::Function fd_2   = robot_model_2dof.forward_dynamics();
    casadi::Function fd_3   = robot_model_2dof_wobase.forward_dynamics();

    casadi::Function id_1   = reduced_robot_model.inverse_dynamics();
    casadi::Function id_2   = robot_model_2dof.inverse_dynamics();
    casadi::Function id_3   = robot_model_2dof_wobase.inverse_dynamics();

    casadi::Function coriolis_1 = reduced_robot_model.coriolis_matrix();
    casadi::Function coriolis_2 = robot_model_2dof.coriolis_matrix();
    casadi::Function coriolis_3 = robot_model_2dof_wobase.coriolis_matrix();

    casadi::Function mass_inverse_1 = reduced_robot_model.mass_inverse_matrix();
    casadi::Function mass_inverse_2 = robot_model_2dof.mass_inverse_matrix();
    casadi::Function mass_inverse_3 = robot_model_2dof_wobase.mass_inverse_matrix();
    casadi::Function mass_inverse_4 = robot_model_7dof.mass_inverse_matrix();

    casadi::Function regressor_1 = reduced_robot_model.joint_torque_regressor();
    casadi::Function regressor_2 = robot_model_2dof.joint_torque_regressor();
    casadi::Function regressor_3 = robot_model_2dof_wobase.joint_torque_regressor();

    std::vector<double> q_vec_random((size_t)reduced_robot_model.n_q);
    Eigen::Map<mecali::ConfigVector>( q_vec_random.data(), q_vec_random.size(), 1 ) = reduced_robot_model.randomConfiguration();
    // std::vector<double> q_vec_random = {-0.54030230586, -0.8414709848, 0.6};
    std::vector<double> q_vec_random_2((size_t)robot_model_7dof.n_q);
    Eigen::Map<mecali::ConfigVector>( q_vec_random_2.data(), q_vec_random_2.size(), 1 ) = robot_model_7dof.randomConfiguration();

    std::vector<double> dq_vec_random((size_t)reduced_robot_model.n_dof);
    Eigen::Map<mecali::ConfigVector>( dq_vec_random.data(), dq_vec_random.size(), 1 ) = Eigen::VectorXd::Random(dq_vec_random.size(), 1);
    std::vector<double> ddq_vec_random((size_t)reduced_robot_model.n_dof);
    Eigen::Map<mecali::ConfigVector>( ddq_vec_random.data(), ddq_vec_random.size(), 1 ) = Eigen::VectorXd::Random(ddq_vec_random.size(), 1);
    std::vector<double> tau_vec_random((size_t)reduced_robot_model.n_dof);
    Eigen::Map<mecali::ConfigVector>( tau_vec_random.data(), tau_vec_random.size(), 1 ) = Eigen::VectorXd::Random(tau_vec_random.size(), 1);

    // Evaluate the function with a casadi::DMVector containing q_vec as input
    casadi::DM pos_res_1 = fk_pos_1(casadi::DMVector {q_vec_random})[0];
    casadi::DM pos_res_2 = fk_pos_2(casadi::DMVector {q_vec_random})[0];
    casadi::DM pos_res_3 = fk_pos_3(casadi::DMVector {q_vec_random})[0];
    std::cout << "fk_pos_1 result with random input        : " << pos_res_1 << std::endl;
    std::cout << "fk_pos_2 result with random input        : " << pos_res_2 << std::endl;
    std::cout << "fk_pos_3 result with random input        : " << pos_res_3 << std::endl;

    casadi::DM fd_res_1 = fd_1(casadi::DMVector {q_vec_random, dq_vec_random, tau_vec_random})[0];
    casadi::DM fd_res_2 = fd_2(casadi::DMVector {q_vec_random, dq_vec_random, tau_vec_random})[0];
    casadi::DM fd_res_3 = fd_3(casadi::DMVector {q_vec_random, dq_vec_random, tau_vec_random})[0];
    std::cout << "fd_1 result with random input        : " << fd_res_1 << std::endl;
    std::cout << "fd_2 result with random input        : " << fd_res_2 << std::endl;
    std::cout << "fd_3 result with random input        : " << fd_res_3 << std::endl;

    casadi::DM id_res_1 = id_1(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    casadi::DM id_res_2 = id_2(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    casadi::DM id_res_3 = id_3(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    std::cout << "id_1 result with random input        : " << id_res_1 << std::endl;
    std::cout << "id_2 result with random input        : " << id_res_2 << std::endl;
    std::cout << "id_3 result with random input        : " << id_res_3 << std::endl;

    casadi::DM regressor_res_1 = regressor_1(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    casadi::DM regressor_res_2 = regressor_2(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    casadi::DM regressor_res_3 = regressor_3(casadi::DMVector {q_vec_random, dq_vec_random, ddq_vec_random})[0];
    std::cout << "regressor_1 result with random input        : " << regressor_res_1 << std::endl;
    std::cout << "regressor_2 result with random input        : " << regressor_res_2 << std::endl;
    std::cout << "regressor_3 result with random input        : " << regressor_res_3 << std::endl;

    casadi::DM coriolis_res_1 = coriolis_1(casadi::DMVector {q_vec_random, dq_vec_random})[0];
    casadi::DM coriolis_res_2 = coriolis_2(casadi::DMVector {q_vec_random, dq_vec_random})[0];
    casadi::DM coriolis_res_3 = coriolis_3(casadi::DMVector {q_vec_random, dq_vec_random})[0];
    std::cout << "coriolis_1 result with random input        : " << coriolis_res_1 << std::endl;
    std::cout << "coriolis_2 result with random input        : " << coriolis_res_2 << std::endl;
    std::cout << "coriolis_3 result with random input        : " << coriolis_res_3 << std::endl;

    casadi::DM mass_inverse_res_1 = mass_inverse_1(casadi::DMVector {q_vec_random})[0];
    casadi::DM mass_inverse_res_2 = mass_inverse_2(casadi::DMVector {q_vec_random})[0];
    casadi::DM mass_inverse_res_3 = mass_inverse_3(casadi::DMVector {q_vec_random})[0];
    std::cout << "mass_inverse_1 result with random input        : " << mass_inverse_res_1 << std::endl;
    std::cout << "mass_inverse_2 result with random input        : " << mass_inverse_res_2 << std::endl;
    std::cout << "mass_inverse_3 result with random input        : " << mass_inverse_res_3 << std::endl;

    casadi::DM mass_inverse_res_4 = mass_inverse_4(casadi::DMVector {q_vec_random_2})[0];
    std::cout << "mass_inverse_4 result with random input        : " << mass_inverse_res_4 << std::endl;


    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If not setting options, function fk_T_1 (or any function) will only be C-code-generated as "first_function.c" (or any other name you set)
      // mecali::generate_code(fk_T_1, "first_function");
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      mecali::Dictionary codegen_options;
      codegen_options["c"]=false;
      codegen_options["save"]=true;
      mecali::generate_code(fk_pos_1, "fk_T", codegen_options);
      mecali::generate_code(fd_1, "fd", codegen_options);
      mecali::generate_code(id_1, "id", codegen_options);
      mecali::generate_code(regressor_1, "regressor", codegen_options);
      mecali::generate_code(coriolis_1, "coriolis", codegen_options);
      mecali::generate_code(mass_inverse_1, "massinv", codegen_options);

      // std::cout << fk_pos_1 << std::endl;
      // std::cout << fd_1 << std::endl;
      // std::cout << id_1 << std::endl;
      // std::cout << regressor_1 << std::endl;
      // std::cout << coriolis_1 << std::endl;
      // std::cout << mass_inverse_1 << std::endl;

      // reduced_robot_model.print_model_data();
      robot_model_2dof.print_model_data();
      // robot_model_2dof_wobase.print_model_data();

}
