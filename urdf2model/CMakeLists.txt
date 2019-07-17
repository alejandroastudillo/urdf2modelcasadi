  # cmake_minimum_required(VERSION 2.8.3)
  CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Set the project name
  project(mecali)

# set LINK command
  IF(WIN32)
    SET(LINK copy_if_different)
  ELSE(WIN32)
    SET(LINK create_symlink)
  ENDIF(WIN32)

# options
  option(DEBUG_MODE "Build unit tests" OFF)
  option(BUILD_UNIT_TESTS "Build unit tests" ON)
  option(BUILD_EXAMPLES "Build examples" ON)

# Set GCC flags
  if(DEBUG_MODE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM -DDEBUG")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC -std=c++11 -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM")
  endif(DEBUG_MODE)

# Set source directories
  set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
  set(ROB_DYN_INTERFACE_DIR ${SRC_DIR})


# for linking directories (as the -L flag in gcc)
  find_library(CASADI_LIB casadi HINTS ${CASADI_DIR}/lib)
  # set(casadi_DIR /home/alejandro/phd_software/casadi_source/build)
  # find_package(casadi REQUIRED)

# Include root folder to the project's include path
  # include_directories(${CMAKE_SOURCE_DIR})

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
  FILE(GLOB_RECURSE HEADERS
       ${PROJECT_SOURCE_DIR}/src/*.hpp
       # ${PROJECT_SOURCE_DIR}/src/*.cpp
       )

  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/mecali")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/mecali/functions")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/mecali/interface")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/mecali/utils")

  SET(HEADERS_)
  FOREACH(header ${HEADERS})
   # MESSAGE( STATUS "HEADER is: " ${header} )
   STRING(REGEX REPLACE "${PROJECT_SOURCE_DIR}/src/" "" header ${header})
   # MESSAGE( STATUS "NEW_HEADER is: " ${header} )
   LIST(APPEND HEADERS_ ${header})
   GET_FILENAME_COMPONENT(headerName ${header} NAME)
   GET_FILENAME_COMPONENT(headerPath ${header} PATH)
   EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
     ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
     ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${header})
   INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
     DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${headerPath}
           PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
  ENDFOREACH(header)
  SET(HEADERS ${HEADERS_})


# Add subdirectories
  add_subdirectory (src)

  if(BUILD_EXAMPLES)
    add_subdirectory (examples)
  endif(BUILD_EXAMPLES)

  if(BUILD_UNIT_TESTS)
    enable_testing ()
    add_subdirectory (tests)
  endif(BUILD_UNIT_TESTS)
