
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was spacemit_mappingConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# This package contains precompiled K3 RISC-V libraries.
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _spacemit_mapping_processor)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
   NOT _spacemit_mapping_processor MATCHES "^(riscv64|rv64[a-z0-9_]*)$")
    set(spacemit_mapping_FOUND FALSE)
    set(spacemit_mapping_NOT_FOUND_MESSAGE "This SDK requires a Linux/riscv64 target toolchain")
    return()
endif()

include(CMakeFindDependencyMacro)
find_dependency(Threads)
set(spacemit_mapping_map_sqlite_FOUND FALSE)
if(OFF AND OFF)
    set(spacemit_mapping_map_sqlite_FOUND TRUE)
    if(NOT spacemit_mapping_FIND_COMPONENTS OR "map_sqlite" IN_LIST spacemit_mapping_FIND_COMPONENTS)
        find_dependency(SQLite3 3.24)
    endif()
endif()
set(spacemit_mapping_rendering_FOUND ON)
if(ON)
    if(NOT spacemit_mapping_FIND_COMPONENTS OR "rendering" IN_LIST spacemit_mapping_FIND_COMPONENTS)
        find_dependency(PkgConfig)
        pkg_check_modules(RM_EGL REQUIRED IMPORTED_TARGET egl)
        pkg_check_modules(RM_GLES REQUIRED IMPORTED_TARGET glesv2)
        pkg_check_modules(RM_GBM REQUIRED IMPORTED_TARGET gbm)
        pkg_check_modules(RM_DRM REQUIRED IMPORTED_TARGET libdrm)
    endif()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/spacemit_mappingTargets.cmake")
set(spacemit_mapping_core_FOUND TRUE)
set(spacemit_mapping_geometry_FOUND TRUE)
set(spacemit_mapping_map_storage_FOUND TRUE)
set(spacemit_mapping_fusion_FOUND TRUE)
set(spacemit_mapping_distance_field_FOUND TRUE)
set(spacemit_mapping_surface_FOUND TRUE)
set(spacemit_mapping_mapping_FOUND TRUE)
set(spacemit_mapping_map_io_FOUND OFF)
check_required_components(spacemit_mapping)
