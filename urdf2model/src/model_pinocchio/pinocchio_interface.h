
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#include "../utils/debug_functions.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <casadi/casadi.hpp>

#include <Eigen/Core>


// init function
void robot_init(std::string filename);
// calculate qdd for f
void qdd_cal(double *q, double *qd, double *qdd, double *tau, int parIdx);

void ForwardKinematics_pin(Eigen::VectorXd q);

void print_model_data();

void execute_tests();

// getters
int get_ndof();
int get_nq();


#endif // PINOCCHIO_INTERFACE_H_INCLUDED
