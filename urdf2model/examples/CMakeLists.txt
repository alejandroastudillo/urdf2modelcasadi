
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
# Complete example
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

## scara example
add_executable( scara_example
               scara_example.cpp
             )
target_link_libraries( scara_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                    )

## KUKA KR60 example
add_executable( kuka_kr60_example
               kuka_kr60_example.cpp
             )
target_link_libraries( kuka_kr60_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                                        )

## KUKA KR5 example
add_executable( kuka_kr5_example
               kuka_kr5_example.cpp
             )
target_link_libraries( kuka_kr5_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                                        )

## Atlas example
add_executable( atlas_example
               atlas_example.cpp
             )
target_link_libraries( atlas_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                                        )


## Rubion example
add_executable( rubion_example
               rubion_example.cpp
             )
target_link_libraries( rubion_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                                        )

## Gen3 float example
add_executable( gen3_float_example
               gen3_float_example.cpp
             )
target_link_libraries( gen3_float_example

                      ${PROJECT_NAME}
                      ${CASADI_LIB}
                      pinocchio::pinocchio
                                        )

## MMO-500 example
add_executable( mmo500_example
              mmo500_example.cpp
              )
target_link_libraries( mmo500_example

       ${PROJECT_NAME}
       ${CASADI_LIB}
       pinocchio::pinocchio
                         )