
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#include "../utils/debug_functions.hpp"

#include <casadi/casadi.hpp>
#include "pinocchio/math/casadi.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"


#include <Eigen/Core>

// init function
void robot_init(std::string filename);
// calculate qdd for f
void qdd_cal(double *q, double *qd, double *qdd, double *tau);

void print_model_data();

void test_casadi_aba();
void test_casadi_rnea();
void test_casadi_fk();

// getters
int get_ndof();
int get_nq();


#endif // PINOCCHIO_INTERFACE_H_INCLUDED
