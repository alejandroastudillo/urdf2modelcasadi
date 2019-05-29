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

const double PI = boost::math::constants::pi<double>();

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

struct Robot_info_struct robot_info; // Declare robot_info of type Robot_info_struct

pinocchio::Model  model;                                      // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
pinocchio::Data   data = pinocchio::Data(model);              // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html


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

void ForwardKinematics_pin(Eigen::VectorXd q)
{
    // apply forward kinematics wrt q. Updates data structure
      pinocchio::forwardKinematics(model,data,q);
    // update the position of each frame contained in the model.
      pinocchio::updateFramePlacements(model, data);
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

//    std::cout << "qdd = " << qdd_Eigen << std::endl;
}


void execute_tests()
{
    // casadi::SX x = casadi::SX::sym("x");
    // casadi::SX y = casadi::SX::sym("y");
    // casadi::Function f("f", {x, y}, {2*x, x/y});
    // std::vector<casadi::DM> f_arg = {3,4};
    // std::cout << "f_kin: " << f(f_arg) << std::endl;

    // get the index of the frame corresponding to the end-effector
      int EE_idx = model.nframes-1; // EE_idx = model.getFrameId("EndEffector"); kinova: EndEffector, abb: joint6-tool0, kuka: iiwa_joint_ee
      std::cout << "\n\tName of the end-effector frame = " << model.frames[EE_idx].name << std::endl << std::endl;

    // robot's home configuration
      Eigen::VectorXd q_home = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
      std::cout << "----- Home configuration --> q = " << q_home.transpose() << std::endl;

      ForwardKinematics_pin(q_home);
      Eigen::Vector3d ee_position_0 = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix_0 = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation_0 = ee_rotmatrix_0.eulerAngles(2, 1, 0);

      std::cout << "\tEE position: \t" << ee_position_0.transpose() << std::endl;
      std::cout << "\n\tEE orientation: " << ee_orientation_0.transpose() << std::endl << std::endl;

    // custom joint configuration
      Eigen::VectorXd q(model.nq);
      q << 0, PI/6, 0, 4*PI/6, 0, -2*PI/6, -PI/2;//; //q << cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(PI/2), sin(PI/2); // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;
      std::cout << "----- Custom joint configuration --> q = " << q.transpose() << std::endl;

      ForwardKinematics_pin(q);
      Eigen::Vector3d ee_position = data.oMf[EE_idx].translation();
      Eigen::Matrix3d ee_rotmatrix = data.oMf[EE_idx].rotation();
      Eigen::Vector3d ee_orientation = ee_rotmatrix.eulerAngles(2, 1, 0);

      std::cout << "\tEE position: \t" << ee_position.transpose() << std::endl;
      std::cout << "\n\tEE orientation: " << ee_orientation.transpose() << std::endl << std::endl;

    // double q0[] = {cos(0), sin(0), PI/6, cos(0), sin(0), 4*PI/6, cos(0), sin(0), -2*PI/6, cos(PI/2), sin(PI/2)};
    // double qm[model.nq] = {0};
    // std::cout << "qm before = " << Eigen::Map<Eigen::VectorXd>(qm, model.nq).transpose() << std::endl;
    // map_joint_angles(q0, qm);
    // std::cout << "qm after = " << Eigen::Map<Eigen::VectorXd>(qm, model.nq).transpose() <<  std::endl;
    // std::cout << "IDX: "<< model.joints[7].idx_q() << std::endl;
}


void print_model_data()
{
    print_indent("Model name = ", robot_info.name, 45);
    print_indent("Size of configuration vector = ", robot_info.n_q, 45);
    print_indent("Number of joints (including universe) = ", robot_info.n_joints, 45);
    print_indent("Number of DoF: ", robot_info.n_dof, 45);
    print_indent("Number of bodies: ", robot_info.n_bodies, 45);
    print_indent("Number of operational frames: ", robot_info.n_frames, 45);
    print_indent("Gravity: ", robot_info.gravity, 45);
    print_indent("Joint torque upper bounds: ", robot_info.joint_torque_limit, 45);
    print_indent("Joint configuration upper bounds: ", robot_info.joint_pos_ub, 45);
    print_indent("Joint configuration lower bounds: ", robot_info.joint_pos_lb, 45);
    print_indent("Joint velocity upped bounds: ", robot_info.joint_vel_limit, 45);

    for (int k=0 ; k<model.njoints ; ++k)
    {
    std::cout << model.names[k] << "\t: "
              << data.oMi[k].translation().transpose() << std::endl;
    }
    for (int k=0 ; k<model.nframes ; ++k)
    {
        std::cout << k << "\t: "
              << model.frames[k].name << "\t\t placement: " << data.oMf[k].translation().transpose() << std::endl;
    }
}

int get_ndof() {return model.nv;}
int get_nq() {return model.nq;}
