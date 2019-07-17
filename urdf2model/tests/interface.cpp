
#define BOOST_TEST_MODULE INTERFACE_TESTS
#include <boost/test/unit_test.hpp>

#include "model_interface.hpp"

using namespace mecali; // TODO: Remove this using namespace. It is better to explicitely put the namespace before each attribute like: mecali::Serial_Robot

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

#ifdef MODELS_DIR
    #define Rob_models_dir MAKE_STR(MODELS_DIR)
#else
    #define Rob_models_dir "../../urdf2model/models"
#endif

std::string filename = Rob_models_dir"/kortex_description/urdf/JACO3_URDF_V11.urdf";

// Typedef
  typedef double                              Scalar;
  typedef casadi::SX                          CasadiScalar;

  typedef pinocchio::ModelTpl<Scalar>         Model;
  typedef Model::Data                         Data;

  typedef pinocchio::ModelTpl<CasadiScalar>   CasadiModel;
  typedef CasadiModel::Data                   CasadiData;

  typedef Model::ConfigVectorType             ConfigVector;
  typedef Model::TangentVectorType            TangentVector;

  typedef CasadiModel::ConfigVectorType       ConfigVectorCasadi;
  typedef CasadiModel::TangentVectorType      TangentVectorCasadi;

BOOST_AUTO_TEST_CASE(FK_pinocchio_casadi)
{
  // Instantiate model and data objects
    Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

  // FK test with robot's home configuration
    int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee

  // Pinocchio
    ConfigVector  q_home = pinocchio::randomConfiguration(model, -3.14159*Eigen::VectorXd::Ones(model.nq), 3.14159*Eigen::VectorXd::Ones(model.nq)); // q_home(model.nq); // Eigen::VectorXd q_home = pinocchio::neutral(model);
    // q_home        << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(-PI/2), sin(-PI/2); // q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2; // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;

    pinocchio::forwardKinematics(     model,  data,   q_home);  // apply forward kinematics wrt q. Updates data structure
    pinocchio::updateFramePlacements( model,  data);            // updates the pose of every frame contained in the model.

  // Interface
    Serial_Robot robot_model;
    robot_model = generate_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;

    casadi::DM pos_res = robot_model.fk_pos(casadi::DMVector {q_vec})[0];

    Data::TangentVectorType pos_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(pos_res).data(), 3,1);

  // Check
    BOOST_CHECK(pos_mat.isApprox(data.oMf[EE_idx].translation()));
}

BOOST_AUTO_TEST_CASE(ABA_pinocchio_casadi)
{
// Instantiate model and data objects
    Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

  // Articulated-Body algorithm (forward dynamics) test with robot's home configuration
  // Pinocchio
    ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    TangentVector v_home(Eigen::VectorXd::Zero(model.nv)); // v_home(Eigen::VectorXd::Random(model.nv));
    TangentVector tau_home(Eigen::VectorXd::Zero(model.nv)); // tau_home(Eigen::VectorXd::Random(model.nv));

    pinocchio::aba(model,data,q_home,v_home,tau_home);

  // Interface
    Serial_Robot robot_model;
    robot_model = generate_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

    casadi::DM ddq_res = robot_model.aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

    Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(), model.nv,1);

  // Check
    BOOST_CHECK(ddq_mat.isApprox(data.ddq));
}

BOOST_AUTO_TEST_CASE(RNEA_pinocchio_casadi)
{
// Instantiate model and data objects
    Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

// Recursive Newton-Euler algorithm (inverse dynamics) test with robot's home configuration
  // Pinocchio
    ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    TangentVector v_home(Eigen::VectorXd::Zero(model.nv));
    TangentVector a_home(Eigen::VectorXd::Zero(model.nv));

    pinocchio::rnea(model,data,q_home,v_home,a_home);

  // Interface
    Serial_Robot robot_model;
    robot_model = generate_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> a_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(a_vec.data(),model.nv,1) = a_home;

    casadi::DM tau_res = robot_model.rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];

    Data::TangentVectorType tau_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(tau_res).data(),model.nv,1);

  // Check
    BOOST_CHECK(tau_mat.isApprox(data.tau));
}
