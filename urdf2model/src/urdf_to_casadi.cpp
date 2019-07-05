#include <casadi/casadi.hpp>
#include "model_pinocchio/pinocchio_interface.h"

using namespace casadi;

int main(int argc, char ** argv)
{
    // Select the URDF file to create the model
    std::string filename = (argc<=1) ? "../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V11.urdf" : argv[1];
        // ../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V10rev.urdf
        // ../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V11.urdf
        // ../urdf2model/robot_descriptions/iiwa_description/urdf/iiwa14.urdf
        // ../urdf2model/robot_descriptions/abb_common/urdf/irb120.urdf

    robot_init(filename);
    // execute_tests();

    test_casadi_aba();
    test_casadi_rnea();
    test_casadi_fk();

    #ifdef DEBUG
      print_model_data();
    #endif

    int n_dof = get_ndof();

    // std::cout << "ndof: " << n_dof << std::endl;

    // Variables
    SX q = SX::sym("q",n_dof,1);
    SX qd = SX::sym("qd",n_dof,1);
    SX qdd = SX::zeros(n_dof,1);
    SX tau = SX::sym("tau",n_dof,1);


    // CasadiModel cas_model;
    // CasadiData cas_data(cas_model);
    // generate_model(cas_model, cas_data, "../urdf2model/robot_descriptions/abb_common/urdf/irb120.urdf");
    //
    // std::cout << "name: " << cas_model.name << std::endl;

    Robot_info_struct robot_model;
    robot_model = generate_model("../urdf2model/robot_descriptions/abb_common/urdf/irb120.urdf");
    std::cout << "robot_model name: " << robot_model.name << std::endl;

    std::cout << "robot_model ABA: " << robot_model.aba << std::endl;

    std::vector<double> q_vec = {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0};
    std::vector<double> v_vec = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> a_vec = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> tau_vec = {0, 0, 0, 0, 0, 0, 0};

    casadi::DM ddq_res = robot_model.aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

    std::cout << "ddq: " << ddq_res << std::endl;

    casadi::DM tau_res = robot_model.rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];

    std::cout << "tau: " << tau_res << std::endl;

    casadi::DM pos_res = robot_model.fk_pos(casadi::DMVector {q_vec})[0];

    std::cout << "EE_pos: " << pos_res << std::endl;

}
