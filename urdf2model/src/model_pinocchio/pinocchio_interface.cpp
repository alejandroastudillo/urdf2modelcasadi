/* Check: https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html
*/
#include "pinocchio_interface.h"

pinocchio::Model model;
// data for the NMPC controller
pinocchio::Data data = pinocchio::Data(model);

void robot_init(std::string filename)
{
    pinocchio::urdf::buildModel(filename,model);

    // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     //model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.8));

    // init for NMPC
    data = pinocchio::Data(model);

    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::Function f("f", {x, y}, {2*x, x/y});
    std::vector<casadi::DM> f_arg = {3,4};
    std::cout << "f_kin: " << f(f_arg) << std::endl;

    Eigen::VectorXd EE_pos = Eigen::VectorXd::Zero(3);


    const double pi = boost::math::constants::pi<double>();
    Eigen::VectorXd q_0(model.nq);
    q_0 << 1, 0, pi/6, 1, 0, 4*pi/6, 1, 0, -2*pi/6, 1, -pi/2; // Eigen::VectorXd q_0(7); q_0 << 0, pi/6, 0, 4*pi/6, 0, -2*pi/6, -pi/2;
    std::cout << "q_0 = " << q_0.transpose() << std::endl;

    //Eigen::VectorXd q = pinocchio::neutral(model);  // pinocchio::randomConfiguration(model);
    //std::cout << "q = " << q.transpose() << std::endl;

    int EE_idx = model.getFrameId("EndEffector");

    ForwardKinematics_pin(q_0);

    std::cout << "\nEE position: " << data.oMf[EE_idx].translation().transpose() << std::endl;
    std::cout << "\nEE rotation matrix: \n" << data.oMf[EE_idx+1].rotation() << std::endl;

    // print_model_data();
    std::cout << "IDX: "<< model.joints[7].idx_q() << std::endl;
}

void ForwardKinematics_pin(Eigen::VectorXd q){
    pinocchio::forwardKinematics(model,data,q);
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

void print_model_data(){
    std::cout << "Number of joints (including universe) = " << data.oMi.size() << std::endl; // model.njoints = data.oMi.size()
    std::cout << "Number of DoF: " << model.nv << std::endl;
    std::cout << "Number of bodies: " << model.nbodies << std::endl;
    std::cout << "Number of operational frames: " << model.nframes << std::endl;
    std::cout << "Upper joint configuration limit: " << model.upperPositionLimit.transpose() << std::endl;
    std::cout << "Vector of maximal joint torques: " << model.effortLimit.transpose() << std::endl;
    std::cout << "Vector of maximal joint velocities: " << model.velocityLimit.transpose() << std::endl;

    for (int k=0 ; k<model.njoints ; ++k)
    std::cout << model.names[k] << "\t: "
              << data.oMi[k].translation().transpose() << std::endl;

    for (int k=0 ; k<model.nframes ; ++k){
        std::cout << k << "\t: "
              << model.frames[k].name << "\t\t placement: " << data.oMf[k].translation().transpose() << std::endl;
    }

}

int get_ndof() {return model.nv;}
int get_nq() {return model.nq;}
