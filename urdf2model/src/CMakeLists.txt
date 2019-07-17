

SET(${PROJECT_NAME}_SOURCES)
LIST(APPEND ${PROJECT_NAME}_SOURCES

    ${ROB_DYN_INTERFACE_DIR}/pinocchio_interface.cpp
    ${SRC_DIR}/functions/forward_dynamics.cpp
    ${SRC_DIR}/functions/inverse_dynamics.cpp
    ${SRC_DIR}/functions/forward_kinematics.cpp
    ${SRC_DIR}/functions/code_generation.cpp
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
ADD_LIBRARY(${PROJECT_NAME} SHARED ${HEADERS} ${${PROJECT_NAME}_SOURCES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
# target link libraries
target_link_libraries(${PROJECT_NAME}

                      stdc++
                      boost_system
                      urdfdom_model
                      ${CASADI_LIB}
                      # casadi
                    )

# Install
# ADD_HEADER_GROUP(HEADERS)
# ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)

# MESSAGE( STATUS "HEADERS is: " ${HEADERS} )
# MESSAGE( STATUS "{{PROJECT_NAME}_SOURCES} is: " ${${PROJECT_NAME}_SOURCES} )
