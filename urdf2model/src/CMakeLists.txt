


# link directories (as the -L flag in gcc)
find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)
# set(casadi_DIR /home/alejandro/phd_software/casadi_source/build)
# find_package(casadi REQUIRED)

# include directories (as the -I flag in gcc)
include_directories(${PINOCCHIO_INC})
include_directories(${EIGEN_INC})
include_directories(${CASADI_DIR}/include)

# link external libraries (as the -l flag in gcc)
add_library(  model_interface
              SHARED
              ${ROB_DYN_INTERFACE_DIR}/pinocchio_interface.cpp
              ${SRC_DIR}/functions/forward_dynamics.cpp
              ${SRC_DIR}/functions/inverse_dynamics.cpp
              ${SRC_DIR}/functions/forward_kinematics.cpp
           )

# target link libraries
target_link_libraries(model_interface

                      stdc++
                      boost_system
                      urdfdom_model
                      ${CASADI_LIB}
                      # casadi
                    )
