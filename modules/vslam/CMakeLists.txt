cmake_minimum_required(VERSION 3.19)
project(spacemit_vslam VERSION 17.0.0 LANGUAGES NONE)

include("${CMAKE_CURRENT_LIST_DIR}/../../tools/binary_sdk.cmake")
spacemit_install_binary_sdk(spacemit_vslam "sdk/riscv64")

install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")
