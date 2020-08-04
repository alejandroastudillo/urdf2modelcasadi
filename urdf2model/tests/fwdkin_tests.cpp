
#define BOOST_TEST_MODULE FWDKIN_TESTS
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
