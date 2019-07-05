

# Adds an executable target to be built from the source files listed after it
add_executable( ${PROJECT_NAME}
                urdf_to_casadi.cpp
              )

# link directories (as the -L flag in gcc)
find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)
# set(casadi_DIR /home/alejandro/phd_software/casadi_source/build)
# find_package(casadi REQUIRED)

# include directories (as the -I flag in gcc)
include_directories(${PINOCCHIO_INC})
include_directories(${EIGEN_INC})
include_directories(${CASADI_DIR}/include)

# link external libraries (as the -l flag in gcc)
add_library(robot_dynamics
            SHARED
            ${ROB_DYN_INTERFACE_DIR}/pinocchio_interface.cpp)

# target link libraries
target_link_libraries(robot_dynamics

                      stdc++
                      boost_system
                      urdfdom_model
                      ${CASADI_LIB}
                      # casadi
                    )

target_link_libraries(${PROJECT_NAME}

                      robot_dynamics
                    )
