/* Check:
https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html
https://coin-or.github.io/CppAD/doc/cppad_eigen.hpp.htm

From Pinocchio interfacing CppAD
https://github.com/stack-of-tasks/pinocchio/blob/f665f3f93669860beb8e80388ed3b52043ddd868/src/math/cppad.hpp

### Pinocchio doc:
https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio.html#a97be7e9cd332a591b1431e30bab2c51e

### Information about types of variables
Eigen::VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1> = pinocchio::ModelTpl<double>::VectorXs
Eigen::Vector3d = Eigen::Matrix<double, 3, 1> = pinocchio::ModelTpl<double>::Vector3
*/

/* TODO:
  - Handle (print error or warning) when the torque, position, or velocity limits are zero.
*/

#include "pinocchio_interface.h"

#include "../utils/debug_functions.hpp"

// const double PI = boost::math::constants::pi<double>();
const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;


// Declare robot_info of type Robot_info_struct
  Robot_info_struct robot_info;

  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, Eigen::Dynamic>  EigenCasadiMatrix;
  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, 1>               EigenCasadiVecXd;
  typedef Eigen::Matrix<CasadiScalar , 3, 1>                            EigenCasadiVec3d;

// Instantiate model and data objects
  Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
  Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

// std::tuple<double, char, std::string> get_student(int id)
// {
//     return std::make_tuple(3.8, 'A', "Lisa Simpson");
// }

// void generate_model(CasadiModel &cas_model, CasadiData &cas_data, std::string file_name)
// {
//     Model         pin_model;
//     pinocchio::urdf::buildModel(file_name,pin_model);
//     model.gravity.linear(pinocchio::Model::gravity981);
//     Data          pin_data = pinocchio::Data(pin_model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html
//
//     cas_model = model.cast<CasadiScalar>();
//     cas_data = CasadiData(cas_model);
//
//     std::cout << "name: " << cas_model.name << std::endl;
// }

void robot_init(std::string filename)
{
    // build the model using the urdf parser
      bool verbose = false;                                   // verbose can be omitted from the buildModel execution: <pinocchio::urdf::buildModel(filename,model)>
      pinocchio::urdf::buildModel(filename,model,verbose);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html

    // Set the gravity applied to the model
      model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));

    // initialize the data structure for the model
      data = pinocchio::Data(model);

    // fill a data structure with some basic information about the robot
      robot_info.name               = model.name; // robot_info.name.assign(model.name);
      robot_info.n_joints           = model.njoints;  // data.oMi.size()
      robot_info.n_q                = model.nq;
      robot_info.n_dof              = model.nv;
      robot_info.n_bodies           = model.nbodies;
      robot_info.n_frames           = model.nframes;
      robot_info.gravity            = model.gravity.linear_impl();
      robot_info.joint_torque_limit = model.effortLimit;
      robot_info.joint_pos_ub       = model.upperPositionLimit;
      robot_info.joint_pos_lb       = model.lowerPositionLimit;
      robot_info.joint_vel_limit    = model.velocityLimit;
}

void test_casadi_aba()
{
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
      // for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
      // {
      //     ddq_sx(k) = casadi_data.ddq[k];
      // }
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

    // Print results
      std::cout << "\n----- Articulated-Body algorithm (forward dynamics) test: " << std::endl;
      std::cout << "* Pinocchio" << std::endl;
      print_indent("     ddq = ", data.ddq, 38);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     ddq = ", ddq_mat, 38);

}

void test_casadi_rnea()
{
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
      // for(Eigen::Index k = 0; k < model.nv; ++k)
      // {
      //   tau_sx(k) = casadi_data.tau[k];
      // }
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

    // Print results
      std::cout << "\n----- Recursive Newton-Euler algorithm (inverse dynamics) test: " << std::endl;
      std::cout << "* Pinocchio " << std::endl;
      print_indent("     tau = ", data.tau, 38);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     tau = ", tau_mat, 38);

}

void test_casadi_fk()
{
  CasadiModel casadi_model = model.cast<CasadiScalar>();
  CasadiData  casadi_data(casadi_model);

  // FK test with robot's home configuration
    int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee
    print_indent("\nName of the end-effector frame = ", model.frames[EE_idx].name, 39);
    // Pinocchio
      ConfigVector  q_home = pinocchio::randomConfiguration(model, -PI*Eigen::VectorXd::Ones(model.nq), PI*Eigen::VectorXd::Ones(model.nq)); // q_home(model.nq); // Eigen::VectorXd q_home = pinocchio::neutral(model);
      // q_home        << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(-PI/2), sin(-PI/2); // q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2; // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;

      pinocchio::forwardKinematics(     model,  data,   q_home);  // apply forward kinematics wrt q. Updates data structure
      pinocchio::updateFramePlacements( model,  data);            // updates the pose of every frame contained in the model.
      // pinocchio::framesForwardKinematics(model,data,q_home);

      Eigen::Vector3d ee_position_0     = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix_0    = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation_0  = ee_rotmatrix_0.eulerAngles(2, 1, 0);

    // Pinocchio + Casadi
      CasadiScalar        q_sx = casadi::SX::sym( "q", model.nq );
      ConfigVectorCasadi  q_casadi( model.nq );
      pinocchio::casadi::copy( q_sx, q_casadi ); // q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

      pinocchio::forwardKinematics(     casadi_model,   casadi_data,    q_casadi);
      pinocchio::updateFramePlacements( casadi_model,   casadi_data);

      // Copy the expression contained in an Eigen::Matrix into a casadi::SX
      CasadiScalar pos_sx(3,1);
      // for(Eigen::Index k = 0; k < 3; ++k)
      // {
      //     pos_sx(k) = casadi_data.oMf[EE_idx].translation()[k];
      // }
      pinocchio::casadi::copy( casadi_data.oMf[EE_idx].translation(), pos_sx );

      casadi::Function    eval_fk( "eval_fk", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx} );

      std::vector<double> q_vec((size_t)model.nq);
      Eigen::Map<ConfigVector>( q_vec.data(), model.nq, 1 ) = q_home;

      casadi::DM pos_res = eval_fk(casadi::DMVector {q_vec})[0];

      Data::TangentVectorType pos_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(pos_res).data(), 3,1);
      // Eigen::VectorXd pos_mat = Eigen::Map<Eigen::VectorXd>(static_cast< std::vector<double> >(pos_res).data(), 3,1);

    // Print results
      std::cout << "\n----- Forward kinematics test: " << std::endl;
      std::cout << "* Pinocchio" << std::endl;
      print_indent("     q = ",               q_home,           38);
      print_indent("     EE position = ",     ee_position_0,    38);
      print_indent("     EE orientation = ",  ee_orientation_0, 38);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     q = ",               q_home,           38);
      print_indent("     EE position = ",     pos_mat,          38);


      // Copy Casadi to EIGEN
      // casadi::SX cs_mat = casadi::SX::sym("A", 3, 4);
      // Eigen::Matrix<casadi::SX, 3, 4> eig_mat;
      //
      // pinocchio::casadi::copy(cs_mat, eig_mat);
      // std::cout << eig_mat << std::endl;

      // Copy EIGEN to Casadi
      // Eigen::Matrix<casadi::SX, 3, 4> eig_mat2;
      // pinocchio::casadi::sym(eig_mat2, "A");
      //
      // casadi::SX cs_mat2;
      //
      // pinocchio::casadi::copy(eig_mat2, cs_mat2);
      // std::cout << cs_mat2 << std::endl;

}

void print_model_data()
{
    std::cout << "\n----- Robot model information: " << std::endl;
    print_indent("Model name = ",                         robot_info.name,               38);
    print_indent("Size of configuration vector = ",       robot_info.n_q,                38);
    print_indent("Number of joints (with universe) = ",   robot_info.n_joints,           38);
    print_indent("Number of DoF = ",                      robot_info.n_dof,              38);
    print_indent("Number of bodies = ",                   robot_info.n_bodies,           38);
    print_indent("Number of operational frames = ",       robot_info.n_frames,           38);
    print_indent("Gravity = ",                            robot_info.gravity,            38);
    print_indent("Joint torque bounds = ",                robot_info.joint_torque_limit, 38);
    print_indent("Joint configuration upper bounds = ",   robot_info.joint_pos_ub,       38);
    print_indent("Joint configuration lower bounds = ",   robot_info.joint_pos_lb,       38);
    print_indent("Joint velocity bounds = ",              robot_info.joint_vel_limit,    38);

    std::cout << "\n----- Placement of each joint in the model: " << std::endl;
    for (int k=0 ; k<model.njoints ; ++k)
    {
        std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) <<  model.names[k] << std::setw(10) << data.oMi[k].translation().transpose() << std::endl;
    }

    std::cout << "\n----- Placement of each frame in the model: " << std::endl;
    for (int k=0 ; k<model.nframes ; ++k)
    {
        std::cout << std::setprecision(3) << std::left << std::setw(5) <<  k  << std::setw(20) << model.frames[k].name << std::setw(10) << data.oMf[k].translation().transpose() << std::endl;
    }
}


void qdd_cal(double *q, double *qd, double *qdd, double *tau)
{
    Eigen::VectorXd q_Eigen   = Eigen::Map<Eigen::VectorXd>(q, model.nv);
    Eigen::VectorXd qd_Eigen  = Eigen::Map<Eigen::VectorXd>(qd,model.nv);
    Eigen::VectorXd qdd_Eigen = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau_Eigen = Eigen::Map<Eigen::VectorXd>(tau,model.nv);

    qdd_Eigen = pinocchio::aba(model,data,q_Eigen,qd_Eigen,tau_Eigen);

    // to double
    Eigen::Map<Eigen::VectorXd>(qdd,model.nv) = qdd_Eigen;


}


int get_ndof() {return model.nv;}
int get_nq() {return model.nq;}
