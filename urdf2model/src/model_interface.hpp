// #pragma once
#ifndef PINOCCHIO_INTERFACE_H_INCLUDED
#define PINOCCHIO_INTERFACE_H_INCLUDED

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR // Needed for using pinocchio with urdfdom

#include <casadi/casadi.hpp>
#include <pinocchio/math/casadi.hpp>
// #include <pinocchio/autodiff/casadi.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "functions/forward_dynamics.hpp"
#include "functions/inverse_dynamics.hpp"
#include "functions/forward_kinematics.hpp"
#include "functions/code_generation.hpp"
#include "functions/common.hpp"
#include "functions/robot_expressions.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>

/*
TODO Check how to include code_generation as a public method in class Serial_Robot: follow the save example https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp
TODO Add fk jacobians and derivatives to Serial_Robot
QUESTION Should the Serial_Robot class be renamed as Robot, Rigid_Body_Chain, Robot_Model? (It is not just for serial robots anymore)
*/

namespace mecali
{
  class Serial_Robot {

    public:
      // Constructor
      Serial_Robot();

      // VARIABLES
      // data variables
      std::string              name;
      int                      n_q;
      int                      n_joints;
      int                      n_dof;
      int                      n_frames;

      std::vector<std::string> joint_names;
      std::vector<std::string> joint_types;
      Eigen::VectorXd          gravity;          // Eigen::Vector3d
      Eigen::VectorXd          joint_torque_limit;
      Eigen::VectorXd          joint_pos_ub;
      Eigen::VectorXd          joint_pos_lb;
      Eigen::VectorXd          joint_vel_limit;
      Eigen::VectorXd          neutral_configuration;
      Eigen::VectorXd          barycentric_params;

      // METHODS
      void                     import_model(std::string filename);
      void                     import_model(std::string filename, Eigen::Vector3d gravity_vector);
      void                     import_model(std::string filename, Eigen::Vector3d gravity_vector, bool verbose);

      void                     import_reduced_model(std::string filename, std::vector<mecali::Index> joints_to_lock_by_index);
      void                     import_reduced_model(std::string filename, std::vector<mecali::Index> joints_to_lock_by_index, Eigen::VectorXd robot_configuration);
      void                     import_reduced_model(std::string filename, std::vector<mecali::Index> joints_to_lock_by_index, Eigen::VectorXd robot_configuration, Eigen::Vector3d gravity_vector);

      void                     import_reduced_model(std::string filename, std::vector<int> joints_to_lock_by_intid);
      void                     import_reduced_model(std::string filename, std::vector<int> joints_to_lock_by_intid, Eigen::VectorXd robot_configuration);
      void                     import_reduced_model(std::string filename, std::vector<int> joints_to_lock_by_intid, Eigen::VectorXd robot_configuration, Eigen::Vector3d gravity_vector);

      void                     import_reduced_model(std::string filename, std::vector<std::string> joints_to_lock_by_name);
      void                     import_reduced_model(std::string filename, std::vector<std::string> joints_to_lock_by_name, Eigen::VectorXd robot_configuration);
      void                     import_reduced_model(std::string filename, std::vector<std::string> joints_to_lock_by_name, Eigen::VectorXd robot_configuration, Eigen::Vector3d gravity_vector);

      void                     generate_json(std::string filename);


      // random configuration methods
      Eigen::VectorXd          randomConfiguration();
      Eigen::VectorXd          randomConfiguration(Eigen::VectorXd lower_bounds, Eigen::VectorXd upper_bounds);
      Eigen::VectorXd          randomConfiguration(std::vector<double> lower_bounds_v, std::vector<double> upper_bounds_v);

      // function methods
      casadi::Function         forward_dynamics();

      casadi::Function         inverse_dynamics();
      casadi::Function         generalized_gravity();
      casadi::Function         coriolis_matrix();
      casadi::Function         mass_inverse_matrix();
      casadi::Function         joint_torque_regressor();

      casadi::Function         forward_dynamics_derivatives(std::string type);
      casadi::Function         forward_dynamics_derivatives();

      casadi::Function         generalized_gravity_derivatives();
      casadi::Function         inverse_dynamics_derivatives(std::string type);
      casadi::Function         inverse_dynamics_derivatives();

      casadi::Function         forward_kinematics(std::string content, std::vector<std::string> frame_names);
      casadi::Function         forward_kinematics(std::string content, std::vector<int> frame_indices);
      casadi::Function         forward_kinematics(std::string content, std::string frame_name);
      casadi::Function         forward_kinematics(std::string content, int frame_index);
      casadi::Function         forward_kinematics(std::string content);
      casadi::Function         forward_kinematics();

      casadi::Function         robot_expressions(std::vector<std::string> frame_names, bool AUGMENT_ODE);

      // debug methods
      void                     print_model_data();

   private:
      // data variables
      int                      _n_bodies;
      Model                    _model;
      CasadiModel              _casadi_model;
  };

}

extern "C" {
    mecali::Serial_Robot* Serial_Robot_new();
    // void SR_import_model(Serial_Robot* robot_model);
    void import_model_new(mecali::Serial_Robot* robot_model, std::string filename);
    std::string name_new(mecali::Serial_Robot* robot_model);
}

#endif // PINOCCHIO_INTERFACE_H_INCLUDED
