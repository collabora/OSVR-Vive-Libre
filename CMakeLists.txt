#
# Libre HTC Vive plugin for OSVR
#
cmake_minimum_required(VERSION 3.1.0)
project(org_osvr_Vive_Libre)

set (org_osvr_Vive_Libre_VERSION_MAJOR 0)
set (org_osvr_Vive_Libre_VERSION_MINOR 2)

# include(CopyImportedTarget)
include(GNUInstallDirs)
include(FindPkgConfig)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(osvr REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

PKG_CHECK_MODULES (HIDAPI REQUIRED hidapi-libusb)
PKG_CHECK_MODULES (ZLIB REQUIRED zlib)
PKG_CHECK_MODULES (JSONCPP REQUIRED jsoncpp)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

function(add_cxxflag FLAG)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}" PARENT_SCOPE)
endfunction(add_cxxflag)

# enable some gcc warnings
add_cxxflag("-Wall -Wno-long-long -pedantic -Wextra -Wundef -Woverflow")
add_cxxflag("-Woverloaded-virtual -Wformat=2 -Wpointer-arith -Wcast-qual")
add_cxxflag("-Wlogical-op -Wredundant-decls")
#add_cxxflag("-Wmissing-declarations")

include_directories(${EIGEN3_INCLUDE_DIR} ${HIDAPI_INCLUDE_DIRS} src)

# Build the plugin
osvr_convert_json(org_osvr_Vive_Libre_json
    config/org_osvr_Vive_Libre.json
    "${CMAKE_CURRENT_SOURCE_DIR}/org_osvr_Vive_Libre_json.h")

set(SOURCES
    src/vl_hidraw.h
    src/vl_hid_reports.h
    src/vl_driver.cpp
    src/vl_driver.h
    src/vl_fusion.cpp
    src/vl_fusion.h
    src/vl_magic.h
    src/vl_messages.h
    src/vl_config.h
    src/vl_config.cpp
    src/vl_math.h
    src/vl_light.h)

add_library(vive-libre SHARED ${SOURCES})
target_link_libraries(vive-libre
    ${HIDAPI_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${OpenCV_LIBRARIES})

set(OSVR_PLUGIN_SOURCES
    src/org_osvr_Vive_Libre.cpp
    ${SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/org_osvr_Vive_Libre_json.h")

osvr_add_plugin(org_osvr_Vive_Libre ${OSVR_PLUGIN_SOURCES})

target_link_libraries(org_osvr_Vive_Libre
    osvrPluginKit
    osvrCommon
    osvrUtil
    ${HIDAPI_LIBRARIES}
    ${ZLIB_LIBRARIES})

# Install files for the plugin.
install(
    FILES
    config/osvr_server_config.vive_libre.sample.json config/HTC_Vive_meshdata.json
    DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/osvrcore/sample-configs)

install(TARGETS vive-libre DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Build tool
add_executable(vivectl tools/vivectl.cpp)
target_link_libraries(vivectl vive-libre)

install(TARGETS vivectl DESTINATION ${CMAKE_INSTALL_BINDIR})

# tests

add_executable(test-minimal-hid tests/minimal-hid.cpp)
target_link_libraries(test-minimal-hid ${HIDAPI_LIBRARIES})

add_executable(test-hid-enumerate tests/hid-enumerate.cpp)
target_link_libraries(test-hid-enumerate ${HIDAPI_LIBRARIES})
