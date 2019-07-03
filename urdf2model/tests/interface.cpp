
#define BOOST_TEST_MODULE ABA_TESTS
#include <boost/test/unit_test.hpp>

#include "pinocchio_interface.h"

std::string filename =  "../../urdf2model/robot_descriptions/kortex_description/urdf/JACO3_URDF_V11.urdf";

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

    CasadiModel casadi_model = model.cast<CasadiScalar>();
    CasadiData  casadi_data(casadi_model);

  // FK test with robot's home configuration
    int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee

  // Pinocchio
    ConfigVector  q_home = pinocchio::randomConfiguration(model, -3.14159*Eigen::VectorXd::Ones(model.nq), 3.14159*Eigen::VectorXd::Ones(model.nq)); // q_home(model.nq); // Eigen::VectorXd q_home = pinocchio::neutral(model);
    // q_home        << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(-PI/2), sin(-PI/2); // q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2; // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;

    pinocchio::forwardKinematics(     model,  data,   q_home);  // apply forward kinematics wrt q. Updates data structure
    pinocchio::updateFramePlacements( model,  data);            // updates the pose of every frame contained in the model.


  // Pinocchio + Casadi
    CasadiScalar        q_sx = casadi::SX::sym( "q", model.nq );
    ConfigVectorCasadi  q_casadi( model.nq );
    pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    pinocchio::forwardKinematics(     casadi_model,   casadi_data,    q_casadi);
    pinocchio::updateFramePlacements( casadi_model,   casadi_data);

    // Copy the expression contained in an Eigen::Matrix into a casadi::SX
    CasadiScalar pos_sx(3,1);
    pinocchio::casadi::copy( casadi_data.oMf[EE_idx].translation(), pos_sx );

    casadi::Function    eval_fk( "eval_fk", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx} );

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;

    casadi::DM pos_res = eval_fk(casadi::DMVector {q_vec})[0];

    Data::TangentVectorType pos_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(pos_res).data(), 3,1);

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

    CasadiModel casadi_model = model.cast<CasadiScalar>();
    CasadiData casadi_data(casadi_model);

// Articulated-Body algorithm (forward dynamics) test with robot's home configuration
  // Pinocchio
    ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    TangentVector v_home(Eigen::VectorXd::Zero(model.nv)); // v_home(Eigen::VectorXd::Random(model.nv));
    TangentVector tau_home(Eigen::VectorXd::Zero(model.nv)); // tau_home(Eigen::VectorXd::Random(model.nv));

    pinocchio::aba(model,data,q_home,v_home,tau_home);

  // Pinocchio + Casadi
    CasadiScalar q_sx = casadi::SX::sym("q", model.nq);
    ConfigVectorCasadi q_casadi(model.nq);
    pinocchio::casadi::copy(q_sx,q_casadi); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
    TangentVectorCasadi v_casadi(model.nv);
    pinocchio::casadi::copy(v_sx,v_casadi); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

    CasadiScalar tau_sx = casadi::SX::sym("tau", model.nv);
    TangentVectorCasadi tau_casadi(model.nv);
    pinocchio::casadi::copy(tau_sx,tau_casadi); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

    pinocchio::aba(casadi_model,casadi_data,q_casadi,v_casadi,tau_casadi);

    CasadiScalar ddq_sx(model.nv,1);
    pinocchio::casadi::copy( casadi_data.ddq, ddq_sx );

    casadi::Function eval_aba("eval_aba", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_sx});

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

    casadi::DM ddq_res = eval_aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

    Data::TangentVectorType ddq_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(), model.nv,1);

    BOOST_CHECK(ddq_mat.isApprox(data.ddq));

    // BOOST_TEST_MESSAGE( "Testing ABA 0101010101010101 :" );
    // // BOOST_CHECK_EQUAL(4, 4);
    // BOOST_CHECK(true);
    //
    // BOOST_TEST_MESSAGE( "Testing ABA 23232323232322323 :" );
    //
    // BOOST_CHECK(true);
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

  CasadiModel casadi_model = model.cast<CasadiScalar>();
  CasadiData casadi_data(casadi_model);

// Recursive Newton-Euler algorithm (inverse dynamics) test with robot's home configuration
  // Pinocchio
    ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    TangentVector v_home(Eigen::VectorXd::Zero(model.nv));
    TangentVector a_home(Eigen::VectorXd::Zero(model.nv));

    pinocchio::rnea(model,data,q_home,v_home,a_home);

  // Pinocchio + Casadi
    CasadiScalar q_sx = casadi::SX::sym("q", model.nq);
    ConfigVectorCasadi q_casadi(model.nq);
    pinocchio::casadi::copy(q_sx,q_casadi); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

    CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
    TangentVectorCasadi v_casadi(model.nv);
    pinocchio::casadi::copy(v_sx,v_casadi); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);


    CasadiScalar a_sx = casadi::SX::sym("a", model.nv);
    TangentVectorCasadi a_casadi(model.nv);
    pinocchio::casadi::copy(a_sx,a_casadi); // a_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(a_sx).data(),model.nv,1);

    pinocchio::rnea(casadi_model,casadi_data,q_casadi,v_casadi,a_casadi);

    casadi::SX tau_sx(model.nv,1);
    pinocchio::casadi::copy( casadi_data.tau, tau_sx );

    casadi::Function eval_rnea("eval_rnea", casadi::SXVector {q_sx, v_sx, a_sx}, casadi::SXVector {tau_sx});

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> a_vec((size_t)model.nv);
    Eigen::Map<TangentVector>(a_vec.data(),model.nv,1) = a_home;
    casadi::DM tau_res = eval_rnea(casadi::DMVector {q_vec,v_vec,a_vec})[0];
    Data::TangentVectorType tau_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(tau_res).data(),model.nv,1);

    BOOST_CHECK(tau_mat.isApprox(data.tau));
}
