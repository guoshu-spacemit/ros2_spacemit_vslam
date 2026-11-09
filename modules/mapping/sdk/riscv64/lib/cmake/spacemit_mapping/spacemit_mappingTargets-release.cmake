#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "spacemit_mapping::core" for configuration "Release"
set_property(TARGET spacemit_mapping::core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_core.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_core.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::core )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::core "${_IMPORT_PREFIX}/lib/libspacemit_mapping_core.so" )

# Import target "spacemit_mapping::geometry" for configuration "Release"
set_property(TARGET spacemit_mapping::geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::geometry PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_geometry.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_geometry.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::geometry )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::geometry "${_IMPORT_PREFIX}/lib/libspacemit_mapping_geometry.so" )

# Import target "spacemit_mapping::map_storage" for configuration "Release"
set_property(TARGET spacemit_mapping::map_storage APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::map_storage PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_map_storage.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_map_storage.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::map_storage )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::map_storage "${_IMPORT_PREFIX}/lib/libspacemit_mapping_map_storage.so" )

# Import target "spacemit_mapping::fusion" for configuration "Release"
set_property(TARGET spacemit_mapping::fusion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::fusion PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_fusion.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_fusion.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::fusion )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::fusion "${_IMPORT_PREFIX}/lib/libspacemit_mapping_fusion.so" )

# Import target "spacemit_mapping::distance_field" for configuration "Release"
set_property(TARGET spacemit_mapping::distance_field APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::distance_field PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_distance_field.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_distance_field.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::distance_field )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::distance_field "${_IMPORT_PREFIX}/lib/libspacemit_mapping_distance_field.so" )

# Import target "spacemit_mapping::surface" for configuration "Release"
set_property(TARGET spacemit_mapping::surface APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::surface PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_surface.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_surface.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::surface )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::surface "${_IMPORT_PREFIX}/lib/libspacemit_mapping_surface.so" )

# Import target "spacemit_mapping::mapping" for configuration "Release"
set_property(TARGET spacemit_mapping::mapping APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::mapping PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_mapping.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_mapping.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::mapping )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::mapping "${_IMPORT_PREFIX}/lib/libspacemit_mapping_mapping.so" )

# Import target "spacemit_mapping::rendering" for configuration "Release"
set_property(TARGET spacemit_mapping::rendering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::rendering PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping_rendering.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping_rendering.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::rendering )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::rendering "${_IMPORT_PREFIX}/lib/libspacemit_mapping_rendering.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
