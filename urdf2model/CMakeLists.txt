  cmake_minimum_required(VERSION 2.8.3)

# Set the project name
  project(urdf2model_casadi)

# options
  option(BUILD_UNIT_TESTS "Build the unit tests" OFF)

# Set the GCC flags
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM -DDEBUG") # -DDEBUG

# Set source directories
  set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
  set(ROB_DYN_INTERFACE_DIR ${SRC_DIR}/model_pinocchio)

# Add subdirectories
  add_subdirectory (src)

  add_subdirectory (examples)

  if(BUILD_UNIT_TESTS)
    enable_testing ()
    add_subdirectory (tests)
  endif(BUILD_UNIT_TESTS)
