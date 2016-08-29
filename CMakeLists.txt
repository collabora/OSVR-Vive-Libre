#
# Libre HTC Vive plugin for OSVR
#
cmake_minimum_required(VERSION 3.1.0)
project(com_osvr_Vive_Libre)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CopyImportedTarget)
include(GNUInstallDirs)
include(FindPkgConfig)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(osvr REQUIRED)
find_package(JsonCpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams filesystem)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

include_directories(/usr/include/hidapi/ ${EIGEN3_INCLUDE_DIR} src)


# Build the plugin

osvr_convert_json(org_osvr_Vive_Libre_json
    org_osvr_Vive_Libre.json
    "${CMAKE_CURRENT_BINARY_DIR}/org_osvr_Vive_Libre_json.h")

osvr_convert_json(com_osvr_Vive_json
    com_osvr_Vive.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Vive_json.h")

set(SOURCES
    src/hidraw.h
    src/vl_driver.cpp
    src/vl_driver.h
    src/vl_fusion.cpp
    src/vl_fusion.h
    src/vl_magic.h
    src/vl_messages.h
    src/vl_controller.h
    src/vl_config.h
    src/vl_config.cpp
    src/vl_hmd.h
    src/vl_math.h)

add_library(vive-libre SHARED ${SOURCES})
target_link_libraries(vive-libre hidapi-libusb z)

set(OSVR_PLUGIN_SOURCES
    src/org_osvr_Vive_Libre.cpp
    ${SOURCES}
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Vive_json.h"
    "${CMAKE_CURRENT_BINARY_DIR}/org_osvr_Vive_Libre_json.h")

osvr_add_plugin(org_osvr_Vive_Libre ${OSVR_PLUGIN_SOURCES})

target_include_directories(org_osvr_Vive_Libre
    PRIVATE
    ${EIGEN3_INCLUDE_DIR})
target_link_libraries(org_osvr_Vive_Libre osvrPluginKit osvrCommon osvrUtil hidapi-libusb z)

# Install files for the plugin.
install(FILES
    osvr_server_config.vive_libre.sample.json
    DESTINATION .)

# Build tool
add_executable(vivectl tools/vivectl.cpp)
target_link_libraries(vivectl vive-libre)
