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

// const double PI = boost::math::constants::pi<double>();
const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;

struct Robot_info_struct {
   std::string      name;
   int              n_q;
   int              n_joints;
   int              n_dof;
   int              n_bodies;
   int              n_frames;
   Eigen::VectorXd  joint_torque_limit;
   Eigen::VectorXd  joint_pos_ub;
   Eigen::VectorXd  joint_pos_lb;
   Eigen::VectorXd  joint_vel_limit;
   Eigen::VectorXd  gravity;          // Eigen::Vector3d
};

// Declare robot_info of type Robot_info_struct
  Robot_info_struct robot_info;

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

  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, Eigen::Dynamic>  EigenCasadiMatrix;
  typedef Eigen::Matrix<CasadiScalar , Eigen::Dynamic, 1>               EigenCasadiVecXd;
  typedef Eigen::Matrix<CasadiScalar , 3, 1>                            EigenCasadiVec3d;

// Instantiate model and data objects
  Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
  Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html


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

void execute_tests()
{
    // casadi::SX x = casadi::SX::sym("x");
    // casadi::SX y = casadi::SX::sym("y");
    // casadi::Function f("f", {x, y}, {2*x, x/y});
    // std::vector<casadi::DM> f_arg = {3,4};
    // std::cout << "f_kin: " << f(f_arg) << std::endl;

    std::cout << "\n----- Forward kinematics test: " << std::endl;

    // get the index of the frame corresponding to the end-effector
      int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee
      print_indent("Name of the end-effector frame = ", model.frames[EE_idx].name, 40);
      // std::cout << "\n\tName of the end-effector frame = " << model.frames[EE_idx].name << std::endl << std::endl;

    // robot's home configuration
      Eigen::VectorXd q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);

      pinocchio::framesForwardKinematics(model,data,q_home);
      Eigen::Vector3d ee_position_0     = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix_0    = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation_0  = ee_rotmatrix_0.eulerAngles(2, 1, 0);

      std::cout << "Home configuration " << std::endl;
      print_indent("     q = ",               q_home,           40);
      print_indent("     EE position = ",     ee_position_0,    40);
      print_indent("     EE orientation = ",  ee_orientation_0, 40);

    // custom joint configuration
      Eigen::VectorXd q(model.nq);
      // q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2;//;
      q << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(-PI/2), sin(-PI/2); // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;

      pinocchio::framesForwardKinematics(model,data,q);
      Eigen::Vector3d ee_position       = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix      = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation    = ee_rotmatrix.eulerAngles(2, 1, 0);

      std::cout << "Custom joint configuration " << std::endl;
      print_indent("     q = ",               q,              40);
      print_indent("     EE position = ",     ee_position,    40);
      print_indent("     EE orientation = ",  ee_orientation, 40);

    // double q0[] = {cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(PI/2), sin(PI/2)};
    // double qm[model.nq] = {0};
    // std::cout << "qm before = " << Eigen::Map<Eigen::VectorXd>(qm, model.nq).transpose() << std::endl;
    // std::cout << "IDX: "<< model.joints[7].idx_q() << std::endl;
}

void test_casadi_aba()
{
  CasadiModel casadi_model = model.cast<CasadiScalar>();
  CasadiData casadi_data(casadi_model);

  // Articulated-Body algorithm (forward dynamics) test with robot's home configuration
    // Pinocchio
      ConfigVector  q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
      TangentVector v_home(Eigen::VectorXd::Zero(model.nv));
      TangentVector tau_home(Eigen::VectorXd::Zero(model.nv));

      pinocchio::aba(model,data,q_home,v_home,tau_home);

    // Pinocchio + Casadi
      CasadiScalar q_sx = casadi::SX::sym("q", model.nq);
      ConfigVectorCasadi q_casadi(model.nq);
      q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

      CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
      TangentVectorCasadi v_casadi(model.nv);
      v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

      CasadiScalar tau_sx = casadi::SX::sym("tau", model.nv);
      TangentVectorCasadi tau_casadi(model.nv);
      tau_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(tau_sx).data(),model.nv,1);

      pinocchio::aba(casadi_model,casadi_data,q_casadi,v_casadi,tau_casadi);

      CasadiScalar ddq_sx(model.nv,1);
      for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
      {
          ddq_sx(k) = casadi_data.ddq[k];
      }
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
      print_indent("     ddq = ", data.ddq, 40);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     ddq = ", ddq_mat, 40);
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
      q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

      CasadiScalar v_sx = casadi::SX::sym("v", model.nv);
      TangentVectorCasadi v_casadi(model.nv);
      v_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(v_sx).data(),model.nv,1);

      CasadiScalar a_sx = casadi::SX::sym("a", model.nv);
      TangentVectorCasadi a_casadi(model.nv);
      a_casadi = Eigen::Map<TangentVectorCasadi>(static_cast< std::vector<CasadiScalar> >(a_sx).data(),model.nv,1);

      pinocchio::rnea(casadi_model,casadi_data,q_casadi,v_casadi,a_casadi);

      casadi::SX tau_sx(model.nv,1);
      for(Eigen::Index k = 0; k < model.nv; ++k)
      {
        tau_sx(k) = casadi_data.tau[k];
      }
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
      print_indent("     tau = ", data.tau, 40);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     tau = ", tau_mat, 40);

}

void test_casadi_fk()
{
  CasadiModel casadi_model = model.cast<CasadiScalar>();
  CasadiData casadi_data(casadi_model);

  // FK test with robot's home configuration
    int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee
    // Pinocchio
      ConfigVector q_home = pinocchio::neutral(model);

      pinocchio::forwardKinematics(model,data,q_home);  // apply forward kinematics wrt q. Updates data structure
      pinocchio::updateFramePlacements(model, data);    // update the position of each frame contained in the model.
      // pinocchio::framesForwardKinematics(model,data,q_home);

      Eigen::Vector3d ee_position_0     = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix_0    = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation_0  = ee_rotmatrix_0.eulerAngles(2, 1, 0);

      Eigen::Vector3d act7_position     = data.oMi[model.njoints-1].translation();

    // Pinocchio + Casadi
      CasadiScalar q_sx = casadi::SX::sym("q", model.nq);
      ConfigVectorCasadi q_casadi(model.nq);
      q_casadi = Eigen::Map<ConfigVectorCasadi>(static_cast< std::vector<CasadiScalar> >(q_sx).data(),model.nq,1);

      pinocchio::forwardKinematics(casadi_model,casadi_data,q_casadi);
      // pinocchio::updateFramePlacements(casadi_model,casadi_data);

      CasadiScalar pos_sx(3,1);
      for(Eigen::Index k = 0; k < 3; ++k)
      {
          pos_sx(k) = casadi_data.oMi[model.njoints-1].translation()[k];
      }

      casadi::Function eval_fk("eval_fk", casadi::SXVector {q_sx}, casadi::SXVector {pos_sx});

      std::vector<double> q_vec((size_t)model.nq);
      Eigen::Map<ConfigVector>(q_vec.data(),model.nq,1) = q_home;

      casadi::DM pos_res = eval_fk(casadi::DMVector {q_vec})[0];

      Data::TangentVectorType pos_mat = Eigen::Map<Data::TangentVectorType>(static_cast< std::vector<double> >(pos_res).data(), 3,1);


    // Print results
      std::cout << "\n----- Forward kinematics test: " << std::endl;
      std::cout << "* Pinocchio" << std::endl;
      print_indent("     q = ",                       q_home,         40);
      print_indent("     Actuator 7 position = ",     act7_position,  40);
      std::cout << "* Pinocchio + Casadi" << std::endl;
      print_indent("     q = ",               q_home,           40);
      print_indent("     Actuator 7 position = ",     pos_mat,    40);

}

void print_model_data()
{
    std::cout << "\n----- Robot model information: " << std::endl;
    print_indent("Model name = ",                         robot_info.name,               40);
    print_indent("Size of configuration vector = ",       robot_info.n_q,                40);
    print_indent("Number of joints (with universe) = ",   robot_info.n_joints,           40);
    print_indent("Number of DoF = ",                      robot_info.n_dof,              40);
    print_indent("Number of bodies = ",                   robot_info.n_bodies,           40);
    print_indent("Number of operational frames = ",       robot_info.n_frames,           40);
    print_indent("Gravity = ",                            robot_info.gravity,            40);
    print_indent("Joint torque bounds = ",                robot_info.joint_torque_limit, 40);
    print_indent("Joint configuration upper bounds = ",   robot_info.joint_pos_ub,       40);
    print_indent("Joint configuration lower bounds = ",   robot_info.joint_pos_lb,       40);
    print_indent("Joint velocity bounds = ",              robot_info.joint_vel_limit,    40);

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

int get_ndof() {return model.nv;}
int get_nq() {return model.nq;}
