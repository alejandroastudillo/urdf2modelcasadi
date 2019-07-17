
# link directories (as the -L flag in gcc)
# find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)

# include directories (as the -I flag in gcc)
include_directories(${PINOCCHIO_INC})
include_directories(${EIGEN_INC})
include_directories(${CASADI_DIR}/include)

# Adds an executable target to be built from the source files listed after it
add_executable( basic_example
                basic_example.cpp
              )
target_link_libraries( basic_example

                       ${PROJECT_NAME}
                     )
