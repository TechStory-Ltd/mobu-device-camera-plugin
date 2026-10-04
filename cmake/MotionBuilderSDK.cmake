# Imports the MotionBuilder OpenReality SDK library (fbsdk) as the `fbsdk` target.
#
# Inputs (cache variables, see the root CMakeLists.txt):
#   MOBU_VERSION       MotionBuilder version, e.g. 2025
#   MOBU_ROOT          MotionBuilder installation folder
#   OPENREALITY_ROOT   OpenReality SDK folder (headers and import libraries)

if(NOT EXISTS "${OPENREALITY_ROOT}/include/fbsdk/fbsdk.h")
	message(FATAL_ERROR
		"OpenReality SDK not found at '${OPENREALITY_ROOT}'.\n"
		"Install the MotionBuilder ${MOBU_VERSION} SDK, or pass -DMOBU_ROOT=<MotionBuilder folder> / -DOPENREALITY_ROOT=<SDK folder>.")
endif()

if(WIN32)
	set(MOBU_BIN_PATH x64)
else()
	set(MOBU_BIN_PATH linux_64)
endif()

add_library(fbsdk SHARED IMPORTED GLOBAL)
set_target_properties(fbsdk PROPERTIES
	IMPORTED_LOCATION "${MOBU_ROOT}/bin/${MOBU_BIN_PATH}/${CMAKE_SHARED_LIBRARY_PREFIX}fbsdk${CMAKE_SHARED_LIBRARY_SUFFIX}"
	IMPORTED_IMPLIB "${OPENREALITY_ROOT}/lib/${MOBU_BIN_PATH}/fbsdk${CMAKE_STATIC_LIBRARY_SUFFIX}"
	INTERFACE_INCLUDE_DIRECTORIES "${OPENREALITY_ROOT}/include"
)

# the SDK headers are built without the Cg support
add_compile_definitions(K_NO_CG)
