find_package (Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

include_directories (${ROB_DYN_INTERFACE_DIR}
                     ${CASADI_DIR}/include
                     ${EIGEN_INC}
                     ${PINOCCHIO_INC}
                     ${Boost_INCLUDE_DIRS}
                     )

find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)

add_definitions (-DBOOST_TEST_DYN_LINK)

add_executable (Basic_tests basictests.cpp)
target_link_libraries (Basic_tests
                       # ${CASADI_LIB}
                       # stdc++
                       ${Boost_FILESYSTEM_LIBRARY}
                       ${Boost_SYSTEM_LIBRARY}
                       ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                       )

add_executable (Interface_tests interface.cpp)
target_link_libraries (Interface_tests

                      ${CASADI_LIB}
                      stdc++
                      boost_system
                      urdfdom_model
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )

add_test (NAME Basic_Test COMMAND Basic_tests)
add_test (NAME Interface_Test COMMAND Interface_tests)
