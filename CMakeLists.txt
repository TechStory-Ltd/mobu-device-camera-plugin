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
)

add_library(device_oscCamera SHARED ${PLUGIN_SOURCES})
target_include_directories(device_oscCamera PRIVATE src)

# the SDK headers pick the ORSDK<version> namespace by this define, it has to match the MotionBuilder it is built for
target_compile_definitions(device_oscCamera PRIVATE PRODUCT_VERSION=${MOBU_VERSION})

# no platform socket libraries needed - the network access goes through FBTCPIP
target_link_libraries(device_oscCamera PRIVATE fbsdk)

if(MSVC)
	target_compile_options(device_oscCamera PRIVATE /WX) # warnings as errors, same as the MotionBuilder samples
endif()

if(COPY_TO_PLUGINS)
	add_custom_command(TARGET device_oscCamera POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:device_oscCamera> "${MOBU_PLUGINS_DIR}/"
	)
endif()

# --- standalone tests ---------------------------------------------------------------------------
# The OSC decoder and the camera math are header-only and SDK free, so they are tested without MotionBuilder.
# Not part of the default build:
#   cmake --build build --target device_oscCamera_decoder_test device_oscCamera_math_test

add_executable(device_oscCamera_decoder_test EXCLUDE_FROM_ALL tests/osc_camera_decoder_test.cpp)
target_include_directories(device_oscCamera_decoder_test PRIVATE src)

add_executable(device_oscCamera_math_test EXCLUDE_FROM_ALL tests/osc_camera_math_test.cpp)
target_include_directories(device_oscCamera_math_test PRIVATE src)
