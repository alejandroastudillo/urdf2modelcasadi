  cmake_minimum_required(VERSION 2.8.3)

# Set the project name
  project(urdf2model_casadi)

# options
  option(DEBUG_MODE "Build unit tests" OFF)
  option(BUILD_UNIT_TESTS "Build unit tests" OFF)
  option(BUILD_EXAMPLES "Build examples" ON)

# Set GCC flags
  if(DEBUG_MODE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM -DDEBUG")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM")
  endif(DEBUG_MODE)
# Set source directories
  set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
  set(ROB_DYN_INTERFACE_DIR ${SRC_DIR}/interface)

# Include root folder to the project's include path
  include_directories(${CMAKE_SOURCE_DIR})

# Add subdirectories
  add_subdirectory (src)

  if(BUILD_EXAMPLES)
    add_subdirectory (examples)
  endif(BUILD_EXAMPLES)

  if(BUILD_UNIT_TESTS)
    enable_testing ()
    add_subdirectory (tests)
  endif(BUILD_UNIT_TESTS)
