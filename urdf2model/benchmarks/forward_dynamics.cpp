#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      // std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/GEN3_URDF_V12.urdf";
      std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/JACO3_URDF_V11lim.urdf";
    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
      // Define (optinal) gravity vector to be used
        Eigen::Vector3d gravity_vector(0,0,-9.81);
      // Create the model based on a URDF file
        robot_model.import_model(urdf_filename, gravity_vector);


    // ---------------------------------------------------------------------
    // Compute ABA solely from Pinocchio wo Integrate
    // ---------------------------------------------------------------------
      // Instantiate model and data objects
        mecali::Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
        mecali::Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

        pinocchio::urdf::buildModel(urdf_filename, model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
      // Set the gravity applied to the model
        model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
      // initialize the data structure for the model
        data = pinocchio::Data(model);

      // Articulated-Body algorithm (forward dynamics) test with robot's home configuration
        mecali::ConfigVector  q_home = pinocchio::randomConfiguration(model); // pinocchio::neutral(model);
        mecali::TangentVector v_home(Eigen::VectorXd::Random(model.nv)); // v_home(Eigen::VectorXd::Zero(model.nv));
        mecali::TangentVector tau_home(Eigen::VectorXd::Random(model.nv)); // tau_home(Eigen::VectorXd::Zero(model.nv));

        pinocchio::aba(model,data,q_home,v_home,tau_home);

      // Save results
        mecali::Data::MatrixXs ddq_ref   = data.ddq;

    // ---------------------------------------------------------------------
    // Compute ABA solely from Pinocchio w/integrate
    // ---------------------------------------------------------------------
        mecali::Data          data_int = pinocchio::Data(model);
        mecali::ConfigVector  q_int_home(Eigen::VectorXd::Zero(model.nq));
        mecali::TangentVector v_int_home(Eigen::VectorXd::Zero(model.nv));

        pinocchio::integrate(model, q_home, v_int_home, q_int_home);

        pinocchio::aba(model, data_int, q_int_home, v_home, tau_home);
      // Save results
        mecali::Data::MatrixXs ddq_ref_int   = data_int.ddq;

        std::cout << "111111111111111111111\n" << ddq_ref << std::endl;
        std::cout << "222222222222222222222\n" << ddq_ref_int << std::endl;



}
