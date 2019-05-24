
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include <Eigen/Core>
#include <casadi/casadi.hpp>

// init function
void robot_init(std::string filename);

// calculate qdd for f
void qdd_cal(double *q, double *qd, double *qdd, double *tau, int parIdx);

// getters
int get_ndof();
int get_nq();

#endif // PINOCCHIO_INTERFACE_H_INCLUDED
