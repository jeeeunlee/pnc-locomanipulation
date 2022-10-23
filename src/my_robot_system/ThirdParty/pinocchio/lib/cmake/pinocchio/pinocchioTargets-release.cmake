#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pinocchio::pinocchio" for configuration "Release"
set_property(TARGET pinocchio::pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/jelee/dependency_ws/pinocchio/build/bin/lib/libpinocchio.so.2.6.10"
  IMPORTED_SONAME_RELEASE "libpinocchio.so.2.6.10"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio "/home/jelee/dependency_ws/pinocchio/build/bin/lib/libpinocchio.so.2.6.10" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
