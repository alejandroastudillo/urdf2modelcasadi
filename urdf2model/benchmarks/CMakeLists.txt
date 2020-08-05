
# link directories (as the -L flag in gcc)
# find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)

# include directories (as the -I flag in gcc)
include_directories(${PINOCCHIO_INC})
include_directories(${EIGEN_INC})
include_directories(${CASADI_DIR}/include)

include_directories(${SRC_DIR})

# Adds an executable target to be built from the source files listed after it
## inverse_dynamics_derivatives
add_executable( inverse_dynamics_derivatives
               inverse_dynamics_derivatives.cpp
             )
target_link_libraries( inverse_dynamics_derivatives

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )
## forward_dynamics_derivatives
add_executable( forward_dynamics_derivatives
               forward_dynamics_derivatives.cpp
             )
target_link_libraries( forward_dynamics_derivatives

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )
