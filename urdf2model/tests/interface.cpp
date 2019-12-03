
#define BOOST_TEST_MODULE INTERFACE_TESTS
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

BOOST_AUTO_TEST_CASE(FK_pinocchio_casadi)
{
  // Instantiate model and data objects
    mecali::Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    mecali::Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

  // FK test with robot's home configuration
    int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee

  // Pinocchio
    mecali::ConfigVector  q_home = pinocchio::randomConfiguration(model, -3.14159*Eigen::VectorXd::Ones(model.nq), 3.14159*Eigen::VectorXd::Ones(model.nq)); // q_home(model.nq); // Eigen::VectorXd q_home = pinocchio::neutral(model);
    // q_home        << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(-PI/2), sin(-PI/2); // q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2; // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;

    pinocchio::forwardKinematics(     model,  data,   q_home);  // apply forward kinematics wrt q. Updates data structure
    pinocchio::updateFramePlacements( model,  data);            // updates the pose of every frame contained in the model.

  // Interface
    mecali::Serial_Robot robot_model;
    robot_model.import_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;

    casadi::Function fk_pos = robot_model.forward_kinematics("position","EndEffector_Link");
    casadi::Function fk_rot = robot_model.forward_kinematics("rotation","EndEffector_Link");

    casadi::DM pos_res = fk_pos(casadi::DMVector {q_vec})[0];
    casadi::DM rot_res = fk_rot(casadi::DMVector {q_vec})[0];

    mecali::Data::TangentVectorType pos_mat = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(pos_res).data(), 3,1);
    Eigen::MatrixXd                 rot_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(rot_res).data(), 3,3);

  // Check
    BOOST_CHECK(pos_mat.isApprox(data.oMf[EE_idx].translation()));
    BOOST_CHECK(rot_mat.isApprox(data.oMf[EE_idx].rotation()));
}

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

BOOST_AUTO_TEST_CASE(RNEA_pinocchio_casadi)
{
  // Instantiate model and data objects
    mecali::Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    mecali::Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

  // Recursive Newton-Euler algorithm (inverse dynamics) test with robot's home configuration
  // Pinocchio
    mecali::ConfigVector  q_home = pinocchio::randomConfiguration(model);  // pinocchio::randomConfiguration(model); pinocchio::neutral(model);
    mecali::TangentVector v_home(Eigen::VectorXd::Zero(model.nv));
    mecali::TangentVector a_home(Eigen::VectorXd::Zero(model.nv));

    pinocchio::rnea(model,data,q_home,v_home,a_home);

  // Interface
    mecali::Serial_Robot robot_model;
    robot_model.import_model(filename);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> a_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(a_vec.data(),model.nv,1) = a_home;

    casadi::Function rnea = robot_model.inverse_dynamics();

    casadi::DM tau_res = rnea(casadi::DMVector {q_vec, v_vec, a_vec})[0];

    mecali::Data::TangentVectorType tau_mat = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(tau_res).data(),model.nv,1);

  // Check
    BOOST_CHECK(tau_mat.isApprox(data.tau));

  // -----------------------------------------------------------------------
  // Joint torque regressor test with robot's home configuration -----------
  // -----------------------------------------------------------------------
  // Pinocchio
    mecali::Data          data_jtr = pinocchio::Data(model);
    pinocchio::computeJointTorqueRegressor(model,data_jtr,q_home,v_home,a_home);

  // Interface
    casadi::Function regressor = robot_model.joint_torque_regressor();
    casadi::DM reg_res = regressor(casadi::DMVector {q_vec, v_vec, a_vec})[0];
    Eigen::MatrixXd reg_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(reg_res).data(),robot_model.n_dof,10*robot_model.n_dof);

    Eigen::VectorXd tau_regressor_mat = reg_mat * robot_model.barycentric_params;
  // Check
    BOOST_CHECK(reg_mat.isApprox(data_jtr.jointTorqueRegressor));
    BOOST_CHECK(tau_regressor_mat.isApprox(data.tau));

  // -----------------------------------------------------------------------
  // Naive inverse dynamics test with robot's home configuration -----------
  // -----------------------------------------------------------------------
  // Pinocchio
    mecali::Data          data_nt = pinocchio::Data(model);
    pinocchio::computeGeneralizedGravity(model,data_nt,q_home);
    Eigen::VectorXd data_gravity = data_nt.g;
    pinocchio::computeCoriolisMatrix(model,data_nt,q_home,v_home);
    Eigen::MatrixXd data_coriolis = data_nt.C;
    pinocchio::computeMinverse(model,data_nt,q_home);
    Eigen::MatrixXd data_minv = data_nt.Minv;

    // M(q)*ddq + C(q, dq)*dq + g(q) = tau
    Eigen::VectorXd tau_naive_nt = data_minv.inverse()*a_home + data_coriolis*v_home + data_gravity;

    BOOST_CHECK(tau_naive_nt.isApprox(data.tau));

    casadi::Function gravity = robot_model.generalized_gravity();
    casadi::Function coriolis = robot_model.coriolis_matrix();
    casadi::Function mass_inverse = robot_model.mass_inverse_matrix();

    casadi::DM gravity_res = gravity(casadi::DMVector {q_vec})[0];
    casadi::DM coriolis_res = coriolis(casadi::DMVector {q_vec, v_vec})[0];
    casadi::DM mass_inverse_res = mass_inverse(casadi::DMVector {q_vec})[0];

    Eigen::VectorXd gravity_mat = Eigen::Map<Eigen::VectorXd>(static_cast< std::vector<double> >(gravity_res).data(),robot_model.n_dof,1);
    Eigen::MatrixXd coriolis_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(coriolis_res).data(),robot_model.n_dof,robot_model.n_dof);
    Eigen::MatrixXd mass_inverse_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(mass_inverse_res).data(),robot_model.n_dof,robot_model.n_dof);

    // M(q)*ddq + C(q, dq)*dq + g(q) = tau
    Eigen::VectorXd tau_naive_cas = mass_inverse_mat.inverse()*a_home + coriolis_mat*v_home + gravity_mat;

    BOOST_CHECK(gravity_mat.isApprox(data_gravity));
    BOOST_CHECK(coriolis_mat.isApprox(data_coriolis));
    BOOST_CHECK(mass_inverse_mat.isApprox(data_minv));
    BOOST_CHECK(tau_naive_cas.isApprox(data.tau));
}
