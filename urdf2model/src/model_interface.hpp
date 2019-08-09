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

/*
TODO Check how to include code_generation as a public method in class Serial_Robot: follow the save example https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp
TODO Add rotation matrix, jacobians, and derivatives to Serial_Robot
*/

namespace mecali
{
  class Serial_Robot {

    public:
      // data variables
      std::string              name;
      int                      n_q;
      int                      n_joints;
      int                      n_dof;

      std::vector<std::string> joint_names;
      std::vector<std::string> joint_types;
      Eigen::VectorXd          gravity;          // Eigen::Vector3d
      Eigen::VectorXd          joint_torque_limit;
      Eigen::VectorXd          joint_pos_ub;
      Eigen::VectorXd          joint_pos_lb;
      Eigen::VectorXd          joint_vel_limit;
      Eigen::VectorXd          neutral_configuration;

      // function variables
      casadi::Function         aba;
      casadi::Function         rnea;
      casadi::Function         fk_pos;
      casadi::Function         fk_rot;

      // methods
      void                     import_model(std::string filename);
      void                     print_model_data();
      Eigen::VectorXd          randomConfiguration();
      Eigen::VectorXd          randomConfiguration(Eigen::VectorXd lower_bounds, Eigen::VectorXd upper_bounds);
      Eigen::VectorXd          randomConfiguration(std::vector<double> lower_bounds_v, std::vector<double> upper_bounds_v);


   private:
      // data variables
      int                      _n_bodies;
      int                      _n_frames;
  };

}

#endif // PINOCCHIO_INTERFACE_H_INCLUDED
