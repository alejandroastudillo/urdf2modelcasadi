  cmake_minimum_required(VERSION 2.8.3)

# Set the project name
  project(urdf2model_casadi)

# Set the GCC flags
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM")

  set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
  set(ROBOT_DYNAMICS_DIR ${SRC_DIR}/model_pinocchio)

# Adds an executable target to be built from the source files listed after it
  add_executable( ${PROJECT_NAME}
                  ${SRC_DIR}/urdf_to_casadi.cpp)

# link directories (as the -L flag in gcc)
  #link_directories(${CASADI_DIR}/lib)
  find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)

# include directories (as the -I flag in gcc)
  include_directories(${PINOCCHIO_INC})
  include_directories(${EIGEN_INC})
  include_directories(${CASADI_DIR}/include)

# link external libraries (as the -l flag in gcc)
  add_library(robot_dynamics SHARED ${ROBOT_DYNAMICS_DIR}/pinocchio_interface.cpp)
  target_link_libraries(robot_dynamics
                        stdc++
                        boost_system
                        urdfdom_model
                        ${CASADI_LIB})

  target_link_libraries(${PROJECT_NAME}
                        # stdc++
                        # boost_system
                        # urdfdom_model
                        # ${CASADI_LIB}
                        robot_dynamics)
