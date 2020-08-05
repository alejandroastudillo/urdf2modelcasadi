#include <casadi/casadi.hpp>
#include "model_interface.hpp"
using namespace std;
int main()
{
    // ---------------------------------------------------------------------
    // Create a model based on a URDF file
    // ---------------------------------------------------------------------
      std::string urdf_filename = "../urdf2model/models/kortex_description/urdf/GEN3_URDF_V12.urdf";
    // Instantiate a Serial_Robot object called robot_model
      mecali::Serial_Robot robot_model;
      // Define (optinal) gravity vector to be used
        Eigen::Vector3d gravity_vector(0,0,-9.81);
      // Create the model based on a URDF file
        robot_model.import_model(urdf_filename, gravity_vector);


    // ---------------------------------------------------------------------
    // Compute ABA derivatives solely from Pinocchio
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

        pinocchio::computeABADerivatives(model, data, q_home, v_home, tau_home);
        data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

      // Save results
        mecali::Data::MatrixXs ddq_dq_ref   = data.ddq_dq;
        mecali::Data::MatrixXs ddq_dv_ref   = data.ddq_dv;
        mecali::Data::MatrixXs ddq_dtau_ref = data.Minv;

    // ---------------------------------------------------------------------
    // Compute ABA derivatives from Pinocchio + Casadi::jacobian
    // ---------------------------------------------------------------------
      // Instantiate model and data objects
        mecali::CasadiModel cas_model = model.cast<mecali::CasadiScalar>();
        mecali::CasadiData  cas_data( cas_model );
      // Set variables
        mecali::CasadiScalar        q_sx = casadi::SX::sym("q", cas_model.nq);
        mecali::CasadiScalar        q_sx_int(cas_model.nq,1);
        mecali::ConfigVectorCasadi  q_casadi(cas_model.nq);
        mecali::ConfigVectorCasadi  q_int_casadi(cas_model.nq);
        pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);


        mecali::CasadiScalar        v_sx_int = casadi::SX::sym("v_inc", model.nv);
        mecali::ConfigVectorCasadi  v_int_casadi(cas_model.nv);
        pinocchio::casadi::copy( v_sx_int, v_int_casadi );

        pinocchio::integrate(cas_model, q_casadi, v_int_casadi, q_int_casadi);
        pinocchio::casadi::copy( q_int_casadi, q_sx_int );

        mecali::CasadiScalar        v_sx = casadi::SX::sym("v", cas_model.nv);
        mecali::TangentVectorCasadi v_casadi(cas_model.nv);
        pinocchio::casadi::copy( v_sx, v_casadi ); // v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

        mecali::CasadiScalar        tau_sx = casadi::SX::sym("tau", cas_model.nv);
        mecali::TangentVectorCasadi tau_casadi(cas_model.nv);
        pinocchio::casadi::copy( tau_sx, tau_casadi ); // tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

      // Call the Articulated-body algorithm
        pinocchio::aba(cas_model, cas_data, q_int_casadi, v_casadi, tau_casadi);

      // Get the result from ABA into an SX
        mecali::CasadiScalar        ddq_sx(cas_model.nv, 1);
        // pinocchio::casadi::copy( cas_data.ddq, ddq_sx );
        for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
            ddq_sx(k) = cas_data.ddq[k];

      // Compute the jacobian with casadi
        casadi::SX ddq_dq   = jacobian(ddq_sx, v_sx_int);
        casadi::SX ddq_dv   = jacobian(ddq_sx, v_sx);
        casadi::SX ddq_dtau = jacobian(ddq_sx, tau_sx);

      // Define the casadi function to be evaluated
        casadi::Function aba_der_wJacobian("aba_der_wJacobian",
                                    casadi::SXVector {q_sx, v_sx_int, v_sx, tau_sx},
                                    casadi::SXVector {ddq_dq, ddq_dv, ddq_dtau},
                                    std::vector<std::string>{"q", "v_int", "v","tau"},
                                    std::vector<std::string>{"ddq_dq", "ddq_dv", "ddq_dtau"});

      // Evaluate the casadi function
        std::vector<double> q_vec((size_t)model.nq);
        Eigen::Map<mecali::ConfigVector>(q_vec.data(),model.nq,1) = q_home;
        std::vector<double> v_int_vec((size_t)model.nv);
        Eigen::Map<mecali::TangentVector>(v_int_vec.data(),model.nv,1).setZero();
        std::vector<double> v_vec((size_t)model.nv);
        Eigen::Map<mecali::TangentVector>(v_vec.data(),model.nv,1) = v_home;
        std::vector<double> tau_vec((size_t)model.nv);
        Eigen::Map<mecali::TangentVector>(tau_vec.data(),model.nv,1) = tau_home;

        casadi::DM ddq_dq_wJacobian_res   = aba_der_wJacobian(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[0];
        casadi::DM ddq_dv_wJacobian_res   = aba_der_wJacobian(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[1];
        casadi::DM ddq_dtau_wJacobian_res = aba_der_wJacobian(casadi::DMVector {q_vec, v_int_vec, v_vec, tau_vec})[2];

      // Save results
        mecali::Data::MatrixXs ddq_dq_wJacobian_mat   = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dq_wJacobian_res).data(),model.nv,model.nv);
        mecali::Data::MatrixXs ddq_dv_wJacobian_mat   = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dv_wJacobian_res).data(),model.nv,model.nv);
        mecali::Data::MatrixXs ddq_dtau_wJacobian_mat = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dtau_wJacobian_res).data(),model.nv,model.nv);

    // ---------------------------------------------------------------------
    // Compute ABA derivatives from the interface
    // ---------------------------------------------------------------------
      // Get functions
        casadi::Function aba = robot_model.forward_dynamics();

        casadi::Function aba_derivatives = robot_model.forward_dynamics_derivatives();


        casadi::Function aba_der_from_jacobian = aba.jacobian();
        // TODO: Corregir este jacobiano para que retorne 7x21 en lugar de 7x25 (continous joints)
        // TODO: Una vez se corrija esto, tambien seria bueno comparar los tiempos vs cuando no se require la corrección. Es decir, utilizando el mismo urdf pero los joints continuos son cambiados por revolute y limitados.

      // Evaluate the functions
        casadi::DM ddq_dq_res   = aba_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[0];
        casadi::DM ddq_dv_res   = aba_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[1];
        casadi::DM ddq_dtau_res = aba_derivatives(casadi::DMVector {q_vec, v_vec, tau_vec})[2];

      // Save results
        mecali::Data::MatrixXs ddq_dq_mat   = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dq_res).data(),model.nv,model.nv);
        mecali::Data::MatrixXs ddq_dv_mat   = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dv_res).data(),model.nv,model.nv);
        mecali::Data::MatrixXs ddq_dtau_mat = Eigen::Map<mecali::Data::MatrixXs>(static_cast< std::vector<double> >(ddq_dtau_res).data(),model.nv,model.nv);


        // std::cout << "ABA ALL = "         << robot_model.forward_dynamics_derivatives() << std::endl;
        // std::cout << "ABA ddq_dq = "      << robot_model.forward_dynamics_derivatives("ddq_dq") << std::endl;
        // std::cout << "ABA ddq_dv = "      << robot_model.forward_dynamics_derivatives("ddq_dv") << std::endl;
        // std::cout << "ABA ddq_dtau = "    << robot_model.forward_dynamics_derivatives("ddq_dtau") << std::endl;
        // std::cout << "ABA jacobian = "    << robot_model.forward_dynamics_derivatives("jacobian") << std::endl;

    // ---------------------------------------------------------------------
    // Benchmark
    // ---------------------------------------------------------------------
      // First of all compare the results
        std::cout << "From Pinocchio:\n" << ddq_dq_ref << std::endl;
        std::cout << "From Pinocchio + Casadi::Jacobian:\n" << ddq_dq_wJacobian_mat << std::endl;
        std::cout << "From Interface:\n" << ddq_dq_mat << std::endl;

        std::cout << "\n\nJacobian from interface: \t" << robot_model.forward_dynamics_derivatives("jacobian") << std::endl;
        std::cout << "\n\nJacobian from CasADi: \t" << aba_der_from_jacobian << std::endl;
    // ---------------------------------------------------------------------
    // Generate (or save) a function
    // ---------------------------------------------------------------------
    // Code-generate or save a function
      // If you use options, you can set if you want to C-code-generate the function, or just save it as "second_function.casadi" (which can be loaded afterwards using casadi::Function::load("second_function.casadi"))
      // mecali::Dictionary codegen_options;
      // codegen_options["c"]=false;
      // codegen_options["save"]=true;
      // mecali::generate_code(fkpos_ee, "kinova_fkpos_ee", codegen_options);
      // mecali::generate_code(fkrot_ee, "kinova_fkrot_ee", codegen_options);
      // mecali::generate_code(fk_ee, "kinova_fk_ee", codegen_options);
      // mecali::generate_code(fk, "kinova_fk", codegen_options);
      // mecali::generate_code(fd, "kinova_fd", codegen_options);
      // mecali::generate_code(id, "kinova_id", codegen_options);



}
