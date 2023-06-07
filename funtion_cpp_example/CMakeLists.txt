cmake_minimum_required(VERSION 2.8.6)
project(casadi_cpp_ex)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_COMPILER "gcc")


#set(CASADI_PATH $ENV{CASADI_DIRECTORY})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/casadi")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
#include_directories("/home/mtplnr/mpc_ws/casadi_source/build/install_folder/include")

link_libraries(casadi)
#link_directories(/home/mtplnr/mpc_ws/casadi_source/build/install_folder/lib)
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib/casadi")

find_package(CASADI REQUIRED)

set( INDY7_CASADI
  indy7_M
  indy7_Minv
  indy7_C
  indy7_G
  indy7_id
  indy7_fd
  indy7_fk
  indy7_fk_ee
  indy7_fkrot_ee
  indy7_J_s
  indy7_J_b
  indy7_J_fd
  indy7_J_id
)

link_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/URDF2CASADI"
)

if(WITH_DL AND WITH_IPOPT)
  add_executable(nlp_codegen nlp_codegen.cpp)
  target_link_libraries(nlp_codegen casadi)
endif()

if(WITH_DL AND NOT WIN32)
  add_executable(codegen_usage codegen_usage.cpp)
  add_executable(c_api_usage c_api_usage.cpp)  
  target_link_libraries(codegen_usage casadi)
  target_compile_definitions(codegen_usage PRIVATE "-DINCLUDE_DIR=\"${PROJECT_SOURCE_DIR}\"")
  target_link_libraries(c_api_usage casadi)
  target_compile_definitions(c_api_usage PRIVATE "-DINCLUDE_DIR=\"${PROJECT_SOURCE_DIR}\"")
endif()

add_executable(casadi_function_load "src/casadi_function_load.cpp")
target_link_libraries(casadi_function_load casadi)
target_compile_definitions(casadi_function_load PRIVATE "-DINCLUDE_DIR=\"${PROJECT_SOURCE_DIR}\"")

