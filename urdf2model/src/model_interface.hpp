// #pragma once
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR // Needed for using pinocchio with urdfdom

#include <casadi/casadi.hpp>
#include <pinocchio/math/casadi.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/crba.hpp"
// #include "pinocchio/algorithm/rnea-derivatives.hpp"
// #include "pinocchio/algorithm/aba-derivatives.hpp"

#include "functions/forward_dynamics.hpp"
#include "functions/inverse_dynamics.hpp"
#include "functions/forward_kinematics.hpp"
#include "functions/code_generation.hpp"
#include "functions/common.hpp"

namespace mecali
{

  struct Serial_Robot {
     std::string              name;
     int                      n_q;
     int                      n_joints;
     int                      n_dof;
     int                      n_bodies;
     int                      n_frames;
     std::vector<std::string> joint_names;
     Eigen::VectorXd          gravity;          // Eigen::Vector3d
     Eigen::VectorXd          joint_torque_limit;
     Eigen::VectorXd          joint_pos_ub;
     Eigen::VectorXd          joint_pos_lb;
     Eigen::VectorXd          joint_vel_limit;
     Eigen::VectorXd          neutral_configuration;
     std::vector<std::string> joint_types;
     casadi::Function         aba;
     casadi::Function         rnea;
     casadi::Function         fk_pos;
     casadi::Function         fk_rot;
     // TODO: Add rotation matrix, jacobians, and derivatives
  };

  Serial_Robot generate_model(std::string filename);

  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model);
  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model, Eigen::VectorXd lower_bounds, Eigen::VectorXd upper_bounds);
  Eigen::VectorXd randomConfiguration(Serial_Robot& rob_model, std::vector<double> lower_bounds_v, std::vector<double> upper_bounds_v);

  void print_model_data(Serial_Robot robot_info);

}

#endif // PINOCCHIO_INTERFACE_H_INCLUDED
