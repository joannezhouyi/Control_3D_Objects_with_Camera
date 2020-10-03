#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PyOpenPose" for configuration "Release"
set_property(TARGET PyOpenPose APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PyOpenPose PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/PyOpenPose.so"
  IMPORTED_SONAME_RELEASE "PyOpenPose.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS PyOpenPose )
list(APPEND _IMPORT_CHECK_FILES_FOR_PyOpenPose "${_IMPORT_PREFIX}/lib/PyOpenPose.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
