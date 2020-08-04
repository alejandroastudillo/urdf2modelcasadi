
#define BOOST_TEST_MODULE RNEA_TESTS
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
    mecali::TangentVector v_home(Eigen::VectorXd::Random(model.nv));
    mecali::TangentVector a_home(Eigen::VectorXd::Random(model.nv));

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

    // Populate the zeros in the lower triangle of the Minv
    data_minv.triangularView<Eigen::StrictlyLower>() = data_minv.transpose().triangularView<Eigen::StrictlyLower>();

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

BOOST_AUTO_TEST_CASE(GRAV_DIFF_pinocchio_casadi)
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
    mecali::Serial_Robot robot_model;
    robot_model.import_model(filename);

    mecali::ConfigVector  q_home = pinocchio::randomConfiguration(model);  // pinocchio::randomConfiguration(model); pinocchio::neutral(model);
    mecali::TangentVector v_home(Eigen::VectorXd::Random(model.nv));
    mecali::TangentVector a_home(Eigen::VectorXd::Random(model.nv));

    pinocchio::rnea(model, data, q_home, v_home, a_home);


    mecali::CasadiModel cas_model = model.cast<mecali::CasadiScalar>();
    mecali::CasadiData cas_data(cas_model);

    mecali::CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    mecali::ConfigVectorCasadi  q_casadi(cas_model.nq);

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;


    pinocchio::casadi::copy( q_sx, q_casadi );

  // -----------------------------------------------------------------------
  // Generalized gravity derivatives ---------------------------------------
  // -----------------------------------------------------------------------
  // Pinocchio
    Eigen::MatrixXd g_partial_dq_sc(model.nv,model.nv);
    g_partial_dq_sc.setZero();
    pinocchio::computeGeneralizedGravityDerivatives(model, data, q_home, g_partial_dq_sc);

  // Interface
    mecali::CasadiData::MatrixXs g_partial_dq(model.nv,model.nv);
    g_partial_dq.setZero();
    pinocchio::computeGeneralizedGravityDerivatives(cas_model, cas_data, q_casadi, g_partial_dq);

    casadi::SX          g_partial_dq_sx( cas_model.nv, cas_model.nv);
    pinocchio::casadi::copy( g_partial_dq, g_partial_dq_sx );

    casadi::Function    generalized_gravity_derivatives("generalized_gravity_derivatives", casadi::SXVector {q_sx}, casadi::SXVector {g_partial_dq_sx});

    casadi::DM gravity_deriv_res_0 = generalized_gravity_derivatives(casadi::DMVector {q_vec})[0];

    Eigen::MatrixXd gravity_deriv_mat_0 = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(gravity_deriv_res_0).data(),robot_model.n_dof,robot_model.n_dof);


    // Interface functions
    casadi::Function gravity_derivatives = robot_model.generalized_gravity_derivatives();

    casadi::DM gravity_deriv_res = gravity_derivatives(casadi::DMVector {q_vec})[0];

    Eigen::MatrixXd gravity_deriv_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(gravity_deriv_res).data(),robot_model.n_dof,robot_model.n_dof);

    BOOST_CHECK(gravity_deriv_mat_0.isApprox(gravity_deriv_mat));
    BOOST_CHECK(gravity_deriv_mat.isApprox(g_partial_dq_sc));
}

BOOST_AUTO_TEST_CASE(RNEA_DIFF_pinocchio_casadi)
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
    mecali::TangentVector v_home(Eigen::VectorXd::Random(model.nv));
    mecali::TangentVector a_home(Eigen::VectorXd::Random(model.nv));

    pinocchio::rnea(model, data, q_home, v_home, a_home);

  // Pinochio Test
    mecali::Serial_Robot robot_model;
    robot_model.import_model(filename);

    mecali::CasadiModel cas_model = model.cast<mecali::CasadiScalar>();
    mecali::CasadiData cas_data(cas_model);

    mecali::CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
    mecali::ConfigVectorCasadi  q_casadi(cas_model.nq);
    mecali::ConfigVectorCasadi  q_int_casadi(cas_model.nq);

    mecali::CasadiScalar        v_sx_int = casadi::SX::sym("v_inc", model.nv);
    mecali::ConfigVectorCasadi  v_int_casadi(cas_model.nv);

    pinocchio::casadi::copy( q_sx, q_casadi );
    pinocchio::casadi::copy( v_sx_int, v_int_casadi );

    pinocchio::integrate(cas_model, q_casadi, v_int_casadi, q_int_casadi); // Integrate a configuration vector for the specified model for a tangent vector during one unit time
    mecali::CasadiScalar q_sx_int(model.nq,1);
    pinocchio::casadi::copy( q_int_casadi, q_sx_int );

    mecali::CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
    mecali::TangentVectorCasadi v_casadi(model.nv);
    // v_casadi = Eigen::Map<mecali::TangentVectorCasadi>(static_cast< std::vector<mecali::CasadiScalar> >(v_sx).data(),model.nv,1);
    pinocchio::casadi::copy( v_sx, v_casadi );

    mecali::CasadiScalar a_sx = casadi::SX::sym("a", model.nv);
    mecali::TangentVectorCasadi a_casadi(model.nv);
    // a_casadi = Eigen::Map<mecali::TangentVectorCasadi>(static_cast< std::vector<mecali::CasadiScalar> >(a_sx).data(),model.nv,1);
    pinocchio::casadi::copy( a_sx, a_casadi );

    pinocchio::rnea(cas_model, cas_data, q_int_casadi, v_casadi, a_casadi);

    mecali::CasadiScalar tau_sx(model.nv,1);

    pinocchio::casadi::copy( cas_data.tau, tau_sx );

    casadi::Function eval_rnea("eval_rnea",
                               casadi::SXVector {q_sx, v_sx_int, v_sx, a_sx},
                               casadi::SXVector {tau_sx});

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;

    std::vector<double> v_int_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>( v_int_vec.data(), model.nv, 1 ).setZero();

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>( v_vec.data(), model.nv, 1 ) = v_home;

    std::vector<double> a_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(a_vec.data(), model.nv, 1 ) = a_home;

    // check return value
    casadi::DM tau_res = eval_rnea(casadi::DMVector { q_vec, v_int_vec, v_vec, a_vec })[0];
    // std::cout << "tau_res = " << tau_res << std::endl;
    Eigen::VectorXd tau_mat = Eigen::Map<Eigen::VectorXd>(static_cast< std::vector<double> >(tau_res).data(),robot_model.n_dof,1);

    BOOST_CHECK(data.tau.isApprox(tau_mat));

  // -----------------------------------------------------------------------
  // Generalized gravity derivatives ---------------------------------------
  // -----------------------------------------------------------------------
  // Pinocchio
    Eigen::MatrixXd g_partial_dq_sc(model.nv,model.nv); g_partial_dq_sc.setZero();
    pinocchio::computeGeneralizedGravityDerivatives(model, data, q_home, g_partial_dq_sc);

  // Interface
    // mecali::CasadiData::MatrixXs g_partial_dq(model.nv,model.nv); g_partial_dq.setZero();
    // pinocchio::computeGeneralizedGravityDerivatives(cas_model,cas_data,q_casadi,g_partial_dq);
    //
    // casadi::SX          g_partial_dq_sx( cas_model.nv, cas_model.nv);
    // pinocchio::casadi::copy( g_partial_dq, g_partial_dq_sx );
    //
    // casadi::Function    generalized_gravity_derivatives("generalized_gravity_derivatives", casadi::SXVector {q_sx}, casadi::SXVector {g_partial_dq_sx});
    casadi::Function gravity_derivatives = robot_model.generalized_gravity_derivatives();

    casadi::DM gravity_deriv_res = gravity_derivatives(casadi::DMVector {q_vec})[0];

    Eigen::MatrixXd gravity_deriv_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(gravity_deriv_res).data(),robot_model.n_dof,robot_model.n_dof);

    BOOST_CHECK(gravity_deriv_mat.isApprox(g_partial_dq_sc));

  // -----------------------------------------------------------------------
  // RNEA derivatives ------------------------------------------------------
  // -----------------------------------------------------------------------
  // Pinocchio
    // compute references
  //   mecali::Data::MatrixXs dtau_dq_ref(model.nv,model.nv), dtau_dv_ref(model.nv,model.nv), dtau_da_ref(model.nv,model.nv);
  //   dtau_dq_ref.setZero(); dtau_dv_ref.setZero(); dtau_da_ref.setZero();
  //
  //   pinocchio::computeRNEADerivatives(model, data, q_home, v_home, a_home, dtau_dq_ref, dtau_dv_ref, dtau_da_ref);
  //   dtau_da_ref.triangularView<Eigen::StrictlyLower>() = dtau_da_ref.transpose().triangularView<Eigen::StrictlyLower>();
  //
  // // Interface
  //   // check with respect to q+dq
  //   casadi::SX dtau_dq = jacobian(tau_sx, v_sx_int);
  //   casadi::Function eval_dtau_dq("eval_dtau_dq",
  //                                 casadi::SXVector {q_sx, v_sx_int, v_sx, a_sx},
  //                                 casadi::SXVector {dtau_dq});
  //
  //   casadi::DM dtau_dq_res = eval_dtau_dq(casadi::DMVector {q_vec, v_int_vec, v_vec, a_vec})[0];
  //   std::vector<double> dtau_dq_vec(static_cast< std::vector<double> >(dtau_dq_res));
  //   BOOST_CHECK(Eigen::Map<mecali::Data::MatrixXs>(dtau_dq_vec.data(),model.nv,model.nv).isApprox(dtau_dq_ref));
  //
  //   // check with respect to v+dv
  //   casadi::SX dtau_dv = jacobian(tau_sx, v_sx);
  //   casadi::Function eval_dtau_dv("eval_dtau_dv",
  //                                 casadi::SXVector {q_sx, v_sx_int, v_sx, a_sx},
  //                                 casadi::SXVector {dtau_dv});
  //
  //   casadi::DM dtau_dv_res = eval_dtau_dv(casadi::DMVector {q_vec, v_int_vec, v_vec, a_vec})[0];
  //   std::vector<double> dtau_dv_vec(static_cast< std::vector<double> >(dtau_dv_res));
  //   BOOST_CHECK(Eigen::Map<mecali::Data::MatrixXs>(dtau_dv_vec.data(), model.nv, model.nv).isApprox(dtau_dv_ref));
  //
  //   // check with respect to a+da
  //   casadi::SX dtau_da = jacobian(tau_sx, a_sx);
  //   casadi::Function eval_dtau_da("eval_dtau_da",
  //                                 casadi::SXVector {q_sx, v_sx_int, v_sx, a_sx},
  //                                 casadi::SXVector {dtau_da});
  //
  //   casadi::DM dtau_da_res = eval_dtau_da(casadi::DMVector {q_vec, v_int_vec, v_vec, a_vec})[0];
  //   std::vector<double> dtau_da_vec(static_cast< std::vector<double> >(dtau_da_res));
  //   BOOST_CHECK(Eigen::Map<mecali::Data::MatrixXs>(dtau_da_vec.data(), model.nv, model.nv).isApprox(dtau_da_ref));
  //
  //   // ---
  //   mecali::CasadiData::MatrixXs dtau_dq_casadi(model.nv,model.nv); dtau_dq_casadi.setZero();
  //   mecali::CasadiData::MatrixXs dtau_dv_casadi(model.nv,model.nv); dtau_dv_casadi.setZero();
  //   mecali::CasadiData::MatrixXs dtau_da_casadi(model.nv,model.nv); dtau_da_casadi.setZero();
  //
  //   pinocchio::computeRNEADerivatives(cas_model,cas_data,q_casadi,v_casadi,a_casadi,dtau_dq_casadi,dtau_dv_casadi,dtau_da_casadi);

    // mecali::CasadiData::MatrixXs dtau_dq_casadi(model.nv,model.nv), dtau_dv_casadi(model.nv,model.nv), dtau_da_casadi(model.nv,model.nv);
    // dtau_dq_casadi.setZero();
    // dtau_dv_casadi.setZero();
    // dtau_da_casadi.setZero();
    // pinocchio::computeRNEADerivatives(cas_model, cas_data, q_casadi, v_casadi, a_casadi, dtau_dq_casadi, dtau_dv_casadi, dtau_da_casadi);



}
