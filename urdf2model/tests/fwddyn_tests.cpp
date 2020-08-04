
#define BOOST_TEST_MODULE ABA_TESTS
#include <boost/test/unit_test.hpp>

#include "model_interface.hpp"

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

#ifdef MODELS_DIR
    #define Rob_models_dir MAKE_STR(MODELS_DIR)
#else
    #define Rob_models_dir "../../urdf2model/models"
#endif

std::string filename = Rob_models_dir"/kortex_description/urdf/JACO3_URDF_V11.urdf";

BOOST_AUTO_TEST_CASE(ABA_pinocchio_casadi)
{
  // Instantiate model and data objects
    mecali::Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    mecali::Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

  // Articulated-Body algorithm (forward dynamics) test with robot's home configuration
  // Pinocchio
    mecali::ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    mecali::TangentVector v_home(Eigen::VectorXd::Zero(model.nv)); // v_home(Eigen::VectorXd::Random(model.nv));
    mecali::TangentVector tau_home(Eigen::VectorXd::Zero(model.nv)); // tau_home(Eigen::VectorXd::Random(model.nv));

    pinocchio::aba(model,data,q_home,v_home,tau_home);

  // Interface
    mecali::Serial_Robot robot_model;
    robot_model.import_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

    casadi::Function aba = robot_model.forward_dynamics();

    casadi::DM ddq_res = aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

    mecali::Data::TangentVectorType ddq_mat = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(), model.nv,1);

  // Check
    BOOST_CHECK(ddq_mat.isApprox(data.ddq));


}
