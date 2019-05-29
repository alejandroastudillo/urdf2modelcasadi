#include <casadi/casadi.hpp>
#include "model_pinocchio/pinocchio_interface.h"

using namespace casadi;

int main()
{
    // Select the URDF file to create the model
        std::string filename = "../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V10rev.urdf";
        // std::string filename = "../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V10.urdf";
        // std::string filename = "../urdf2model/robot_descriptions/iiwa_description/urdf/iiwa14.urdf";
        // std::string filename = "../urdf2model/robot_descriptions/abb_common/urdf/irb120.urdf";

    robot_init(filename);
    execute_tests();

    #ifdef DEBUG
      print_model_data();
    #endif

    int n_dof = get_ndof();
    int n_q = get_nq();

    // std::cout << "ndof: " << n_dof << std::endl;

    // Variables
    SX q = SX::sym("q",n_dof,1);
    SX qd = SX::sym("qd",n_dof,1);
    SX qdd = SX::zeros(n_dof,1);
    SX tau = SX::sym("tau",n_dof,1);
}
