cmake_minimum_required(VERSION 3.19)

project(mobu_device_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MotionBuilder version to build against and where it is installed
set(MOBU_VERSION 2025 CACHE STRING "MotionBuilder version")
set(MOBU_ROOT "C:/Program Files/Autodesk/MotionBuilder ${MOBU_VERSION}" CACHE PATH "MotionBuilder installation folder")
set(OPENREALITY_ROOT "${MOBU_ROOT}/OpenRealitySDK" CACHE PATH "OpenReality SDK folder")

# Copy the built plugin into a MotionBuilder plugins folder after every build.
# The default folder is under Program Files, so building to it needs administrator rights.
set(COPY_TO_PLUGINS OFF CACHE BOOL "Copy the plugin into the MotionBuilder plugins folder after the build")
set(MOBU_PLUGINS_DIR "${MOBU_ROOT}/bin/x64/plugins" CACHE PATH "Destination of COPY_TO_PLUGINS")

include(cmake/MotionBuilderSDK.cmake)

# --- the version ---------------------------------------------------------------------------------
# The plugin version is <PLUGIN_VERSION_MAJOR>.<PLUGIN_VERSION_MINOR>.<build number>, the build number grows by one at every build of the plugin (every MotionBuilder version
# counts on the same number). The counter is kept in build_number.txt next to this file (not in git) and the header with both is
# written to the build folder; the About box of the device shows them.
set(PLUGIN_VERSION_MAJOR 1 CACHE STRING "Plugin major version")
set(PLUGIN_VERSION_MINOR 0 CACHE STRING "Plugin minor version")
set(BUILD_NUMBER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/build_number.txt")
set(VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/osc_camera_version.h")
# the same version for the installer: setup_package/Setup.iss includes this file (not in git)
set(VERSION_INNO "${CMAKE_CURRENT_SOURCE_DIR}/setup_package/version.iss")

add_custom_target(oscCamera_version
	COMMAND ${CMAKE_COMMAND} -DBUILD_NUMBER_FILE=${BUILD_NUMBER_FILE} -DOUTPUT_HEADER=${VERSION_HEADER} -DOUTPUT_INNO=${VERSION_INNO}
		-DVERSION_MAJOR=${PLUGIN_VERSION_MAJOR} -DVERSION_MINOR=${PLUGIN_VERSION_MINOR} -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildNumber.cmake"
	BYPRODUCTS "${VERSION_HEADER}" "${VERSION_INNO}"
	COMMENT "Plugin version ${PLUGIN_VERSION_MAJOR}.${PLUGIN_VERSION_MINOR}: next build number"
	VERBATIM
)

# --- the plugin ---------------------------------------------------------------------------------

set(PLUGIN_SOURCES
	src/device_osccamera.cxx
	src/device_osccamera_device.cxx
	src/device_osccamera_device.h
	src/device_osccamera_hardware.cxx
	src/device_osccamera_hardware.h
	src/device_osccamera_layout.cxx
	src/device_osccamera_layout.h
	src/osc_camera_decoder.h
	src/osc_camera_math.h
	src/osc_camera_timeline.h
	src/osc_marker_link.h
)

add_library(device_oscCamera SHARED ${PLUGIN_SOURCES})
target_include_directories(device_oscCamera PRIVATE src "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_dependencies(device_oscCamera oscCamera_version)

# the SDK headers pick the ORSDK<version> namespace by this define, it has to match the MotionBuilder it is built for
target_compile_definitions(device_oscCamera PRIVATE PRODUCT_VERSION=${MOBU_VERSION})

# no platform socket libraries needed - the network access goes through FBTCPIP
target_link_libraries(device_oscCamera PRIVATE fbsdk)

if(MSVC)
	target_compile_options(device_oscCamera PRIVATE /WX) # warnings as errors, same as the MotionBuilder samples
endif()

# Every build leaves the plugin in bin/<version>/, the input of setup_package/Setup.iss (build_all.bat does the same copy)
add_custom_command(TARGET device_oscCamera POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/bin/${MOBU_VERSION}"
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:device_oscCamera> "${CMAKE_CURRENT_SOURCE_DIR}/bin/${MOBU_VERSION}/"
	COMMENT "Copy the plugin to bin/${MOBU_VERSION}"
	VERBATIM
)

if(COPY_TO_PLUGINS)
	add_custom_command(TARGET device_oscCamera POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:device_oscCamera> "${MOBU_PLUGINS_DIR}/"
	)
endif()

# --- standalone tests ---------------------------------------------------------------------------
# The OSC decoder and the camera math are header-only and SDK free, so they are tested without MotionBuilder.
# Not part of the default build:
#   cmake --build build --target device_oscCamera_decoder_test device_oscCamera_math_test device_oscCamera_timeline_test device_oscCamera_marker_test device_oscCamera_reference_test

add_executable(device_oscCamera_decoder_test EXCLUDE_FROM_ALL tests/osc_camera_decoder_test.cpp)
target_include_directories(device_oscCamera_decoder_test PRIVATE src)

add_executable(device_oscCamera_math_test EXCLUDE_FROM_ALL tests/osc_camera_math_test.cpp)
target_include_directories(device_oscCamera_math_test PRIVATE src)

add_executable(device_oscCamera_timeline_test EXCLUDE_FROM_ALL tests/osc_camera_timeline_test.cpp)
target_include_directories(device_oscCamera_timeline_test PRIVATE src)
target_compile_definitions(device_oscCamera_timeline_test PRIVATE OSC_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")

add_executable(device_oscCamera_marker_test EXCLUDE_FROM_ALL tests/osc_marker_link_test.cpp)
target_include_directories(device_oscCamera_marker_test PRIVATE src)

add_executable(device_oscCamera_reference_test EXCLUDE_FROM_ALL tests/osc_camera_reference_test.cpp)
target_include_directories(device_oscCamera_reference_test PRIVATE src)
