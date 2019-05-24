#include <casadi/casadi.hpp>
#include "kinova_pinocchio/kinova3.h"

using namespace casadi;

int main()
{
    std::string filename = "../urdf2model/src/kortex_description/urdf/JACO3_URDF_V10.urdf";
    kinova3_init(filename);

    int n_dof = get_ndof();
    int n_q = get_nq();

    std::cout << "ndof: " << n_dof << std::endl;

    // Variables
    SX q = SX::sym("q",n_dof,1);
    SX qd = SX::sym("qd",n_dof,1);
    SX qdd = SX::zeros(n_dof,1);
    SX tau = SX::sym("tau",n_dof,1);

    SX x = SX::sym("x");
    SX y = SX::sym("y");
    Function f("f", {x, y}, {2*x, x/y});
    std::vector<DM> f_arg = {3,4};
    std::cout << "f: " << f(f_arg) << std::endl;

}
