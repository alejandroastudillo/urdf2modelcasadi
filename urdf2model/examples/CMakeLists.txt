
# link directories (as the -L flag in gcc)
# find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)

# include directories (as the -I flag in gcc)
include_directories(${PINOCCHIO_INC})
include_directories(${EIGEN_INC})
include_directories(${CASADI_DIR}/include)

include_directories(${SRC_DIR})

# Adds an executable target to be built from the source files listed after it
# Basic example
add_executable( basic_example
                basic_example.cpp
              )
target_link_libraries( basic_example

                       ${PROJECT_NAME}
                       ${CASADI_LIB}
                       pinocchio::pinocchio
                     )
# # Complete example
add_executable( complete_example
               complete_example.cpp
             )
target_link_libraries( complete_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

# Kinova example
add_executable( kinova_example
                kinova_example.cpp
              )
target_link_libraries( kinova_example

                       ${PROJECT_NAME}
                       ${CASADI_LIB}
                       pinocchio::pinocchio
                     )

add_executable( kinova_2dof_example
               kinova_2dof_example.cpp
             )
target_link_libraries( kinova_2dof_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

# # UR10 example
add_executable( ur10_example
               ur10_example.cpp
             )
target_link_libraries( ur10_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

# # Yumi example
add_executable( yumi_example
               yumi_example.cpp
             )
target_link_libraries( yumi_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )
# # TJ example
add_executable( tj_example
                tj_example.cpp
              )
target_link_libraries( tj_example

                       ${PROJECT_NAME}
                       ${CASADI_LIB}
                       pinocchio::pinocchio
                     )
# # # Gen3 example
add_executable( gen3_example
               gen3_example.cpp
             )
target_link_libraries( gen3_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

# ## Kinova with gripper example
add_executable( kinova_gripper_example
               kinova_gripper_example.cpp
             )
target_link_libraries( kinova_gripper_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

## iiwa example
add_executable( iiwa_example
               iiwa_example.cpp
             )
target_link_libraries( iiwa_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )
