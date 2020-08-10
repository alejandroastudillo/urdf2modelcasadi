
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

std::string filename = Rob_models_dir"/kortex_description/urdf/JACO3_URDF_V11lim.urdf";

BOOST_AUTO_TEST_CASE(ABA_pinocchio_casadi)
{
  // ---------------------------------------------------------------------
  // 1.1 Compute ABA solely from Pinocchio wo Integrate
  // ---------------------------------------------------------------------
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

  // ---------------------------------------------------------------------
  // 1.2 Compute ABA solely from Pinocchio w/integrate
  // ---------------------------------------------------------------------
      mecali::Data          data_int = pinocchio::Data(model);
      mecali::ConfigVector  q_int_home(Eigen::VectorXd::Zero(model.nq));
      mecali::TangentVector v_int_home(Eigen::VectorXd::Zero(model.nv));

      pinocchio::integrate(model, q_home, v_int_home, q_int_home);

      pinocchio::aba(model, data_int, q_int_home, v_home, tau_home);
    // Save results
      mecali::Data::MatrixXs ddq_ref_int   = data_int.ddq;

      BOOST_CHECK((data_int.ddq).isApprox(data.ddq));

  // ---------------------------------------------------------------------
  // 2.1 Compute ABA from Pinocchio+Casadi wo Integrate
  // ---------------------------------------------------------------------
      mecali::Serial_Robot robot_model;
      robot_model.import_model(filename);

      mecali::CasadiModel cas_model = model.cast<mecali::CasadiScalar>();
      mecali::CasadiData cas_data(cas_model);

      // Set variables
      mecali::CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
      mecali::ConfigVectorCasadi  q_casadi(cas_model.nq);
      pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

      mecali::CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
      mecali::TangentVectorCasadi v_casadi(cas_model.nv);
      pinocchio::casadi::copy( v_sx, v_casadi ); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

      mecali::CasadiScalar        tau_sx = casadi::SX::sym("tau", cas_model.nv);
      mecali::TangentVectorCasadi tau_casadi(cas_model.nv);
      pinocchio::casadi::copy( tau_sx, tau_casadi ); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

      // Call the Articulated-body algorithm
      pinocchio::aba(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);

      // Get the result from ABA into an SX
      mecali::CasadiScalar        ddq_sx(cas_model.nv, 1);
      pinocchio::casadi::copy( cas_data.ddq, ddq_sx );

      // Create the ABA function
      casadi::Function    aba_2_1("aba_2_1", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {ddq_sx});

      std::vector<double> q_vec((size_t)model.nq);
      Eigen::Map<mecali::ConfigVector>(q_vec.data(),model.nq,1) = q_home;

      std::vector<double> v_vec((size_t)model.nv);
      Eigen::Map<mecali::TangentVector>(v_vec.data(),model.nv,1) = v_home;

      std::vector<double> tau_vec((size_t)model.nv);
      Eigen::Map<mecali::TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

      // casadi::Function aba = robot_model.forward_dynamics();
      casadi::DM ddq_res_2_1 = aba_2_1(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

      mecali::Data::TangentVectorType ddq_mat_2_1 = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res_2_1).data(), model.nv,1);

      BOOST_CHECK(ddq_mat_2_1.isApprox(data_int.ddq));

  // ---------------------------------------------------------------------
  // 2.2 Compute ABA from Pinocchio+Casadi w/Integrate
  // ---------------------------------------------------------------------
      mecali::CasadiData cas_data_int(cas_model);

      mecali::ConfigVectorCasadi  q_int_casadi(cas_model.nq);

      mecali::CasadiScalar        v_sx_int = casadi::SX::sym("v_inc", model.nv);
      mecali::ConfigVectorCasadi  v_int_casadi(cas_model.nv);
      pinocchio::casadi::copy( v_sx_int, v_int_casadi );

      pinocchio::integrate(cas_model, q_casadi, v_int_casadi, q_int_casadi); // Integrate a configuration vector for the specified model for a tangent vector during one unit time
      mecali::CasadiScalar q_sx_int(model.nq,1);
      pinocchio::casadi::copy( q_int_casadi, q_sx_int );

      std::vector<double> v_int_vec((size_t)model.nv);
      // Eigen::Map<mecali::TangentVector>(v_int_vec.data(),model.nv,1) = v_home;
      Eigen::Map<mecali::TangentVector>(v_int_vec.data(),model.nv,1).setZero();

      pinocchio::aba(cas_model, cas_data_int, q_int_casadi, v_casadi, tau_casadi);

      casadi::SX ddq_sx_2_2(model.nv,1);
      for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
        ddq_sx_2_2(k) = cas_data_int.ddq[k];
      casadi::Function eval_aba("eval_aba",
                                casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                                casadi::SXVector {ddq_sx_2_2});
      casadi::DM ddq_res_2_2 = eval_aba(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];

      mecali::Data::TangentVectorType ddq_mat_2_2 = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res_2_2).data(), model.nv,1);

      BOOST_CHECK(ddq_mat_2_2.isApprox(ddq_mat_2_1));


      // mecali::CasadiScalar        v_int_sx_zeros = casadi::SX::sym("v_int_zeros", Sparsity::dense(model.nv, 1));
      // mecali::CasadiScalar        v_int_sx_zeros = casadi::SX::zeros(v_sx.sparsity());
      // y = MX.sym('y',Sparsity.lower(n));
      // casadi:DM v_int_sx_zeros = casadi::DM::zeros(model.nv, 1);

      // casadi::Function eval_aba_simp("eval_aba_simp",
      //                           casadi::SXVector {q_sx, v_sx, tau_sx},
      //                           casadi::SXVector {eval_aba(casadi::SXVector {q_sx, v_int_sx_zeros, v_sx, tau_sx})});
      // casadi::DM ddq_res_2_2_simp = eval_aba_simp(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      // mecali::Data::TangentVectorType ddq_mat_2_2_simp = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res_2_2_simp).data(), model.nv,1);
      // BOOST_CHECK(ddq_mat_2_2_simp.isApprox(ddq_mat_2_2));


  // ---------------------------------------------------------------------
  // 3.1 Compute ABA from Interface wo Integrate
  // ---------------------------------------------------------------------
      casadi::Function aba = robot_model.forward_dynamics();

      casadi::DM ddq_res_interface = aba(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

      mecali::Data::TangentVectorType ddq_mat = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res_interface).data(), model.nv,1);

    // Check
      BOOST_CHECK(ddq_mat.isApprox(ddq_mat_2_2));


  // ---------------------------------------------------------------------
  // 4.1 Compute ABA Jacobian from Pinocchio+Casadi w/Integrate
  // ---------------------------------------------------------------------


}

BOOST_AUTO_TEST_CASE(ABA_DIFF_pinocchio_casadi)
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
    mecali::ConfigVector  q_home = pinocchio::randomConfiguration(model); // pinocchio::neutral(model);
    mecali::TangentVector v_home(Eigen::VectorXd::Random(model.nv)); // v_home(Eigen::VectorXd::Zero(model.nv));
    mecali::TangentVector tau_home(Eigen::VectorXd::Random(model.nv)); // tau_home(Eigen::VectorXd::Zero(model.nv));

    pinocchio::aba(model,data,q_home,v_home,tau_home);

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

    std::vector<double> q_vec((size_t)model.nq);
    Eigen::Map<mecali::ConfigVector>(q_vec.data(),model.nq,1) = q_home;

    std::vector<double> v_int_vec((size_t)model.nv);
    // Eigen::Map<mecali::TangentVector>(v_int_vec.data(),model.nv,1) = v_home;
    Eigen::Map<mecali::TangentVector>(v_int_vec.data(),model.nv,1).setZero();

    mecali::CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
    mecali::TangentVectorCasadi v_casadi(model.nv);
    v_casadi = Eigen::Map<mecali::TangentVectorCasadi>(static_cast< std::vector<mecali::CasadiScalar> >(v_sx).data(),model.nv,1);
    // pinocchio::casadi::copy( v_sx, v_casadi );

    mecali::CasadiScalar tau_sx = casadi::SX::sym("tau", model.nv);
    mecali::TangentVectorCasadi tau_casadi(model.nv);
    tau_casadi = Eigen::Map<mecali::TangentVectorCasadi>(static_cast< std::vector<mecali::CasadiScalar> >(tau_sx).data(),model.nv,1);
    // pinocchio::casadi::copy( a_sx, a_casadi );

    pinocchio::aba(cas_model, cas_data, q_int_casadi, v_casadi, tau_casadi);

    casadi::SX ddq_sx(model.nv,1);
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
      ddq_sx(k) = cas_data.ddq[k];
    casadi::Function eval_aba("eval_aba",
                              casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                              casadi::SXVector {ddq_sx});

    std::vector<double> v_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(v_vec.data(),model.nv,1) = v_home;

    std::vector<double> tau_vec((size_t)model.nv);
    Eigen::Map<mecali::TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

    // casadi::Function aba = robot_model.forward_dynamics();
    casadi::DM ddq_res = eval_aba(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];

    mecali::Data::TangentVectorType ddq_mat = Eigen::Map<mecali::Data::TangentVectorType>(static_cast< std::vector<double> >(ddq_res).data(), model.nv,1);

  // Check
    BOOST_CHECK(ddq_mat.isApprox(data.ddq));

  // DERIVATIVES
  // compute references
    mecali::Data::MatrixXs ddq_dq_ref(model.nv,model.nv), ddq_dv_ref(model.nv,model.nv), ddq_dtau_ref(model.nv,model.nv);
    ddq_dq_ref.setZero(); ddq_dv_ref.setZero(); ddq_dtau_ref.setZero();

    pinocchio::computeABADerivatives(model, data, q_home, v_home, tau_home, ddq_dq_ref, ddq_dv_ref, ddq_dtau_ref);
    ddq_dtau_ref.triangularView<Eigen::StrictlyLower>() = ddq_dtau_ref.transpose().triangularView<Eigen::StrictlyLower>();

  // Interface

  // check with respect to q+dq
    casadi::SX ddq_dq = jacobian(ddq_sx, v_sx_int);
    casadi::Function eval_ddq_dq("eval_ddq_dq",
                                  casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                                  casadi::SXVector {ddq_dq});

    casadi::DM ddq_dq_res = eval_ddq_dq(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
    std::vector<double> ddq_dq_vec(static_cast< std::vector<double> >(ddq_dq_res));
    mecali::Data::MatrixXs ddq_dq_res_jac = Eigen::Map<mecali::Data::MatrixXs>(ddq_dq_vec.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dq_res_jac.isApprox(ddq_dq_ref));

  // check with respect to v+dv
    casadi::SX ddq_dv = jacobian(ddq_sx, v_sx);
    casadi::Function eval_ddq_dv("eval_ddq_dv",
                                  casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                                  casadi::SXVector {ddq_dv});

    casadi::DM ddq_dv_res = eval_ddq_dv(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
    std::vector<double> ddq_dv_vec(static_cast< std::vector<double> >(ddq_dv_res));
    mecali::Data::MatrixXs ddq_dv_res_jac = Eigen::Map<mecali::Data::MatrixXs>(ddq_dv_vec.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dv_res_jac.isApprox(ddq_dv_ref));

  // check with respect to tau
    casadi::SX ddq_dtau = jacobian(ddq_sx, tau_sx);
    casadi::Function eval_ddq_dtau("eval_ddq_dtau",
                                  casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                                  casadi::SXVector {ddq_dtau});

    casadi::DM ddq_dtau_res = eval_ddq_dtau(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
    std::vector<double> ddq_dtau_vec(static_cast< std::vector<double> >(ddq_dtau_res));
    mecali::Data::MatrixXs ddq_dtau_res_jac = Eigen::Map<mecali::Data::MatrixXs>(ddq_dtau_vec.data(),model.nv,model.nv);
    BOOST_CHECK(ddq_dtau_res_jac.isApprox(ddq_dtau_ref));

    pinocchio::computeABADerivatives(cas_model, cas_data, q_casadi, v_casadi, tau_casadi);

    cas_data.Minv.triangularView<Eigen::StrictlyLower>() = cas_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

    // call ABA derivatives in Casadi
    casadi::SX ddq_dq_sx(model.nv,model.nv);
    casadi::SX ddq_dv_sx(model.nv,model.nv);
    casadi::SX ddq_dtau_sx(model.nv,model.nv);

    pinocchio::casadi::copy(cas_data.ddq_dq, ddq_dq_sx);
    pinocchio::casadi::copy(cas_data.ddq_dv, ddq_dv_sx);
    pinocchio::casadi::copy(cas_data.Minv, ddq_dtau_sx);

    casadi::Function eval_aba_derivatives_dq("eval_aba_derivatives_dq",
                                              casadi::SXVector {q_sx, v_sx, tau_sx},
                                              casadi::SXVector {ddq_dq_sx});

    casadi::DM ddq_dq_res_direct = eval_aba_derivatives_dq(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    mecali::Data::MatrixXs ddq_dq_res_direct_map = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dq_res_direct).data(),model.nv,model.nv);

    BOOST_CHECK(ddq_dq_ref.isApprox(ddq_dq_res_direct_map));
    BOOST_CHECK(ddq_dq_res_jac.isApprox(ddq_dq_res_direct_map));

    casadi::Function eval_aba_derivatives_dv("eval_aba_derivatives_dv",
                                              casadi::SXVector {q_sx, v_sx, tau_sx},
                                              casadi::SXVector {ddq_dv_sx});

    casadi::DM ddq_dv_res_direct = eval_aba_derivatives_dv(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    mecali::Data::MatrixXs ddq_dv_res_direct_map = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dv_res_direct).data(),model.nv,model.nv);

    BOOST_CHECK(ddq_dv_ref.isApprox(ddq_dv_res_direct_map));
    BOOST_CHECK(ddq_dv_res_jac.isApprox(ddq_dv_res_direct_map));

    casadi::Function eval_aba_derivatives_dtau("eval_aba_derivatives_dtau",
                                              casadi::SXVector {q_sx, v_sx, tau_sx},
                                              casadi::SXVector {ddq_dtau_sx});

    casadi::DM ddq_dtau_res_direct = eval_aba_derivatives_dtau(casadi::DMVector {q_vec,v_vec,tau_vec})[0];
    mecali::Data::MatrixXs ddq_dtau_res_direct_map = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dtau_res_direct).data(),model.nv,model.nv);

    BOOST_CHECK(ddq_dtau_ref.isApprox(ddq_dtau_res_direct_map));
    BOOST_CHECK(ddq_dtau_res_jac.isApprox(ddq_dtau_res_direct_map));

  // Interface functions
    casadi::Function forward_dynamics_derivatives = robot_model.forward_dynamics_derivatives();

    casadi::DM ddq_dq_dm = forward_dynamics_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
    Eigen::MatrixXd ddq_dq_dm_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dq_dm).data(),robot_model.n_dof,robot_model.n_dof);

    casadi::DM ddq_dv_dm = forward_dynamics_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[1];
    Eigen::MatrixXd ddq_dv_dm_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dv_dm).data(),robot_model.n_dof,robot_model.n_dof);

    casadi::DM ddq_dtau_dm = forward_dynamics_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[2];
    Eigen::MatrixXd ddq_dtau_dm_mat = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dtau_dm).data(),robot_model.n_dof,robot_model.n_dof);

    BOOST_CHECK(ddq_dq_dm_mat.isApprox(ddq_dq_res_direct_map));
    BOOST_CHECK(ddq_dv_dm_mat.isApprox(ddq_dv_res_direct_map));
    BOOST_CHECK(ddq_dtau_dm_mat.isApprox(ddq_dtau_res_direct_map));

  // Test partial derivatives
    casadi::Function interface_ddq_dq = robot_model.forward_dynamics_derivatives("ddq_dq");

    // std::cout << "*************** " << interface_ddq_dq(std::vector<casadi::SX>{q_sx, v_sx, tau_sx}) << std::endl;
    // casadi::SX res_ddq_sx = eval_aba(std::vector<casadi::SX>{q_sx, v_sx_int, v_sx, tau_sx});
    // casadi::SX pincas_ddq_dq_sx = jacobian(res_ddq_sx, q_sx);
    // casadi::Function pincas_ddq_dq("pincas_ddq_dq", casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx}, casadi::SXVector {pincas_ddq_dq_sx} );
    //

    // CALCULATE PARTIAL DERIVATIVES OF ABA COMING FROM INTERFACE (using Jacobian)
      casadi::Function interface_aba = robot_model.forward_dynamics();
      casadi::SX interface_aba_sx = interface_aba(std::vector<casadi::SX>{q_sx, v_sx, tau_sx})[0];

      casadi::SX jac_int_ddq_dq_sx    = jacobian(interface_aba_sx, q_sx);
      casadi::SX jac_int_ddq_dv_sx    = jacobian(interface_aba_sx, v_sx);
      casadi::SX jac_int_ddq_dtau_sx  = jacobian(interface_aba_sx, tau_sx);

      casadi::Function jacobian_interface_ddq_dq("jacobian_interface_ddq_dq", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {jac_int_ddq_dq_sx});
      casadi::Function jacobian_interface_ddq_dv("jacobian_interface_ddq_dv", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {jac_int_ddq_dv_sx});
      casadi::Function jacobian_interface_ddq_dtau("jacobian_interface_ddq_dtau", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {jac_int_ddq_dtau_sx});

      casadi::SX full_Jddq_interface_sx = jacobian(interface_aba_sx, vertcat(q_sx,v_sx,tau_sx));
      casadi::Function full_Jddq_interface("full_Jddq_interface", casadi::SXVector {q_sx, v_sx, tau_sx}, casadi::SXVector {full_Jddq_interface_sx});

      //Evaluate
      casadi::DM ddq_dq_dm_0 = jacobian_interface_ddq_dq(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      casadi::DM ddq_dv_dm_0 = jacobian_interface_ddq_dv(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      casadi::DM ddq_dtau_dm_0 = jacobian_interface_ddq_dtau(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
      // casadi::DM Jddq_dm_0 = full_Jddq_interface(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
      casadi::DM Jddq_dm_0 = full_Jddq_interface(casadi::DMVector {q_vec, v_vec, tau_vec})[0];

      Eigen::MatrixXd ddq_dq_mat_0    = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dq_dm_0).data(),robot_model.n_dof,robot_model.n_dof);
      Eigen::MatrixXd ddq_dv_mat_0    = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dv_dm_0).data(),robot_model.n_dof,robot_model.n_dof);
      Eigen::MatrixXd ddq_dtau_mat_0  = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(ddq_dtau_dm_0).data(),robot_model.n_dof,robot_model.n_dof);

      Eigen::MatrixXd Jddq_mat_0  = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(Jddq_dm_0).data(),robot_model.n_dof,3*robot_model.n_dof);

      Eigen::MatrixXd Jddq_mat_0_cat(robot_model.n_dof, 3*robot_model.n_dof);
      Jddq_mat_0_cat << ddq_dq_mat_0, ddq_dv_mat_0, ddq_dtau_mat_0;

      // std::cout << "*** PARTIAL DERIVATIVES: \n" << ddq_dq_mat_0 << "\n\n" << ddq_dv_mat_0 << "\n\n" << ddq_dtau_mat_0 << std::endl;
      // std::cout << "\n*** JACOBIAN: \n" << Jddq_dm_0 << std::endl;
      // std::cout << "*** PARTIAL DERIVATIVES: \n" << ddq_dq_mat_0 << "\n\n" << ddq_dv_mat_0 << "\n\n" << ddq_dtau_mat_0 << std::endl;
      // std::cout << "\n*** JACOBIAN 1: \n" << Jddq_mat_0 << std::endl;

      BOOST_CHECK(Jddq_mat_0.isApprox(Jddq_mat_0_cat));

      casadi::Function full_Jddq_aba_jacobian = interface_aba.jacobian();
      casadi::DM Jddq_dm_from_casadi = full_Jddq_aba_jacobian(casadi::DMVector {q_vec, v_vec, tau_vec, v_int_vec })[0];
      Eigen::MatrixXd Jddq_dm_from_casadi_mat_0  = Eigen::Map<Eigen::MatrixXd>(static_cast< std::vector<double> >(Jddq_dm_from_casadi).data(),robot_model.n_dof,3*robot_model.n_dof);

      BOOST_CHECK(Jddq_mat_0.isApprox(Jddq_dm_from_casadi_mat_0));

      // std::cout << "*** JACOBIANS: \n" << interface_ddq_dq << "\n" << pincas_ddq_dq << "\n" << jacobian_interface_ddq_dq << std::endl;
      // std::cout << full_Jddq_aba_jacobian << std::endl;

}
