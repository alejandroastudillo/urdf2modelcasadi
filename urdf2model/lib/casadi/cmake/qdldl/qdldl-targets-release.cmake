#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qdldl::qdldlstatic" for configuration "Release"
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "/home/user/Fatrop_ws/casadi_source/casadi/lib/lib/libqdldl.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS qdldl::qdldlstatic )
list(APPEND _IMPORT_CHECK_FILES_FOR_qdldl::qdldlstatic "/home/user/Fatrop_ws/casadi_source/casadi/lib/lib/libqdldl.a" )

# Import target "qdldl::qdldl" for configuration "Release"
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/user/Fatrop_ws/casadi_source/casadi/lib/lib/libqdldl.so"
  IMPORTED_SONAME_RELEASE "libqdldl.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS qdldl::qdldl )
list(APPEND _IMPORT_CHECK_FILES_FOR_qdldl::qdldl "/home/user/Fatrop_ws/casadi_source/casadi/lib/lib/libqdldl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
