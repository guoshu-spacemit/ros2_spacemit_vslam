cmake_minimum_required(VERSION 3.22)
project(spacemit_mapping VERSION 0.1.0 LANGUAGES NONE)

include("${CMAKE_CURRENT_LIST_DIR}/../../tools/binary_sdk.cmake")
spacemit_install_binary_sdk(spacemit_mapping "sdk/riscv64")

install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")
