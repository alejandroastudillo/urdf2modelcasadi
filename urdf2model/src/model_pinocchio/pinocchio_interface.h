
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <Eigen/Core>
#include <casadi/casadi.hpp>

// init function
void robot_init(std::string filename);

// calculate qdd for f
void qdd_cal(double *q, double *qd, double *qdd, double *tau, int parIdx);

void ForwardKinematics_pin(Eigen::VectorXd q);

// getters
int get_ndof();
int get_nq();

void print_model_data();
void execute_tests();

#endif // PINOCCHIO_INTERFACE_H_INCLUDED
