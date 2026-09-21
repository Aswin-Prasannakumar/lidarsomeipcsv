#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vsomeip3" for configuration "Release"
set_property(TARGET vsomeip3 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vsomeip3 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vsomeip3.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vsomeip3.dll"
  )

list(APPEND _cmake_import_check_targets vsomeip3 )
list(APPEND _cmake_import_check_files_for_vsomeip3 "${_IMPORT_PREFIX}/lib/vsomeip3.lib" "${_IMPORT_PREFIX}/bin/vsomeip3.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
