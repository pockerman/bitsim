CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")

# version.txt is the only place the version is kept. release-please updates
# it when it makes a release, so do not change it by hand
FILE(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" BITRL_VERSION LIMIT_COUNT 1)
IF(NOT BITRL_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
	MESSAGE(FATAL_ERROR "version.txt must hold a version such as 1.2.3, not '${BITRL_VERSION}'")
ENDIF()
SET(BITRL_VERSION_MAJOR ${CMAKE_MATCH_1})
SET(BITRL_VERSION_MINOR ${CMAKE_MATCH_2})
SET(BITRL_VERSION_PATCH ${CMAKE_MATCH_3})
SET_PROPERTY(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
MESSAGE(STATUS "bitrllib Version ${BITRL_VERSION}")

# create a CMake project
PROJECT(bitrl VERSION ${BITRL_VERSION} LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Prevent in-source builds.
# -----------------------------------------------------------------------------

IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
	MESSAGE( FATAL_ERROR  "In-source build is not possible. Choose an empty directory for build output.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

# Be sure to avoid troubles with library paths when using old policy
IF(COMMAND cmake_policy)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0048 NEW)
ENDIF(COMMAND cmake_policy)

# configure packages. These can be overridden from the command line
# e.g. cmake -DENABLE_CHRONO=OFF -DENABLE_TESTS=ON
OPTION(ENABLE_EXAMPLES "Build the examples" ON)
OPTION(ENABLE_TESTS "Build the unit tests" OFF)
OPTION(ENABLE_OPENCV "Build with OpenCV support" ON)
OPTION(ENABLE_MQTT "Build with MQTT support" ON)
OPTION(ENABLE_CHRONO "Build with Project Chrono support" ON)
OPTION(BITRL_LOG "Log various messages from the library" ON)
OPTION(BITRL_IRRLICHT "Use Irrlicht for visualization. Requires ENABLE_CHRONO" ON)
OPTION(BITRL_WERROR "Treat compiler warnings as errors in Debug builds" OFF)
OPTION(BITRL_SANITIZE "Build bitrl, its tests and examples with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

IF(NOT ENABLE_CHRONO)
	SET(BITRL_IRRLICHT OFF)
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type" FORCE)
ENDIF()

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(BITRL_DEBUG ON)
ELSE()
	SET(BITRL_DEBUG OFF)
ENDIF()

INCLUDE(GNUInstallDirs)
INCLUDE(CMakePackageConfigHelpers)

# Without -DCMAKE_INSTALL_PREFIX bitrl installs into the source tree, as it
# always did: install/opt for a Release build and install/dbg for a Debug build
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	IF(BITRL_DEBUG)
		SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install/dbg" CACHE PATH "Install prefix" FORCE)
	ELSE()
		SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install/opt" CACHE PATH "Install prefix" FORCE)
	ENDIF()
ENDIF()

SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The warnings for bitrl's own code: the library and the tests
FUNCTION(BITRL_SET_WARNINGS target)
	TARGET_COMPILE_OPTIONS(${target} PRIVATE -Wall -Wpedantic)
	IF(BITRL_WERROR AND BITRL_DEBUG)
		TARGET_COMPILE_OPTIONS(${target} PRIVATE -Werror)
	ENDIF()
ENDFUNCTION()

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find packages that we need
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Boost 1.74.0 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

IF(ENABLE_MQTT)
	FIND_PACKAGE(PahoMqttCpp REQUIRED)
	SET(BITRL_MQTT ON)
ENDIF()

IF(ENABLE_OPENCV)
	FIND_PACKAGE(OpenCV REQUIRED)
	SET(BITRL_OPENCV ON)
ENDIF()

IF(ENABLE_TESTS)
	FIND_PACKAGE(GTest REQUIRED)
ENDIF()

IF(ENABLE_CHRONO)
	IF(BITRL_IRRLICHT)
		FIND_PACKAGE(Chrono REQUIRED COMPONENTS Irrlicht)
	ELSE()
		FIND_PACKAGE(Chrono REQUIRED)
	ENDIF()
	SET(BITRL_CHRONO ON)
	SET(BITRL_CHRONO_TARGETS ${CHRONO_TARGETS})
ENDIF()

# -----------------------------------------------------------------------------
# Generated headers. They go into the build directory
# -----------------------------------------------------------------------------

# Earlier builds wrote these into src/. Remove them so they cannot shadow the
# generated ones
FILE(REMOVE ${PROJECT_SOURCE_DIR}/src/bitrl/bitrl_config.h ${PROJECT_SOURCE_DIR}/src/bitrl/bitrl_version.h)

SET(BITRL_GENERATED_DIR ${PROJECT_BINARY_DIR}/generated)

# The robot, sensor and world files. bitrl::consts::data_dir() picks one of
# these at run time
SET(BITRL_DATA_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/bitrl)
SET(BITRL_SOURCE_DATA_DIR ${PROJECT_SOURCE_DIR})
SET(BITRL_INSTALL_DATA_DIR ${CMAKE_INSTALL_FULL_DATADIR}/bitrl)

CONFIGURE_FILE(config.h.in ${BITRL_GENERATED_DIR}/bitrl/bitrl_config.h @ONLY)
CONFIGURE_FILE(version.h.in ${BITRL_GENERATED_DIR}/bitrl/bitrl_version.h @ONLY)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Boost include: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Install dst: ${CMAKE_INSTALL_PREFIX}")

# -----------------------------------------------------------------------------
# The library
# -----------------------------------------------------------------------------

FILE(GLOB SRCS CONFIGURE_DEPENDS
		src/bitrl/*.cpp
		src/bitrl/actuators/*.cpp
		src/bitrl/actuators/backends/*.cpp
		src/bitrl/boards/arduino/*.cpp
		src/bitrl/control/*.cpp
		src/bitrl/dynamics/*.cpp
		src/bitrl/estimation/*.cpp
		src/bitrl/geometry/*.cpp
		src/bitrl/envs/*.cpp
		src/bitrl/envs/gymnasium/*.cpp
		src/bitrl/envs/gymnasium/toy_text/*.cpp
		src/bitrl/envs/gymnasium/classic_control/*.cpp
		src/bitrl/envs/gymnasium/classic_control/vector/*.cpp
		src/bitrl/envs/gdrl/*.cpp
		src/bitrl/envs/multi_armed_bandits/*.cpp
		src/bitrl/envs/grid_world/*.cpp
		src/bitrl/envs/connect2/*.cpp
		src/bitrl/envs/gymnasium/box2d/*.cpp
		src/bitrl/kinematics/*.cpp
		src/bitrl/network/*.cpp
		src/bitrl/planning/*.cpp
		src/bitrl/rigid_bodies/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/impl/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/*.cpp
		src/bitrl/sensors/*.cpp
		src/bitrl/sensors/messages/*.cpp
		src/bitrl/sensors/backends/*.cpp
		src/bitrl/systems/*.cpp
		src/bitrl/trajectory/*.cpp
		src/bitrl/utils/*.cpp
		src/bitrl/utils/io/*.cpp
		src/bitrl/utils/maths/statistics/distributions/*.cpp
		src/bitrl/utils/render/*.cpp
		src/bitrl/worlds/*.cpp
		src/bitrl/worlds/chrono_worlds/*.cpp
   )

ADD_LIBRARY(bitrllib SHARED ${SRCS})
ADD_LIBRARY(bitrl::bitrl ALIAS bitrllib)

SET_TARGET_PROPERTIES(bitrllib PROPERTIES
	LINKER_LANGUAGE CXX
	EXPORT_NAME bitrl
	VERSION ${BITRL_VERSION}
	SOVERSION ${BITRL_VERSION_MAJOR}
	# The installed library finds the libraries it links to that are not in
	# a system directory, Chrono for example, where they were at build time
	INSTALL_RPATH_USE_LINK_PATH TRUE)

BITRL_SET_WARNINGS(bitrllib)

IF(BITRL_SANITIZE)
	# Only for the build tree: an installed bitrl does not force the
	# sanitizers on the projects that use it
	SET(BITRL_SANITIZER_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
	TARGET_COMPILE_OPTIONS(bitrllib PUBLIC $<BUILD_INTERFACE:${BITRL_SANITIZER_FLAGS}>)
	TARGET_LINK_OPTIONS(bitrllib PUBLIC $<BUILD_INTERFACE:${BITRL_SANITIZER_FLAGS}>)
ENDIF()

# Everything that links against bitrllib inherits these
TARGET_COMPILE_FEATURES(bitrllib PUBLIC cxx_std_20)
TARGET_COMPILE_DEFINITIONS(bitrllib PUBLIC BOOST_LOG_DYN_LINK)
TARGET_INCLUDE_DIRECTORIES(bitrllib PUBLIC
	$<BUILD_INTERFACE:${BITRL_GENERATED_DIR}>
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
TARGET_LINK_LIBRARIES(bitrllib PUBLIC Boost::headers boost_log Threads::Threads Eigen3::Eigen)

IF(BITRL_CHRONO)
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC Chrono::Chrono_core)
	IF(BITRL_IRRLICHT)
		TARGET_LINK_LIBRARIES(bitrllib PUBLIC Chrono::Chrono_irrlicht)
		# Chrono's Irrlicht target does not carry the Irrlicht headers
		TARGET_INCLUDE_DIRECTORIES(bitrllib PUBLIC /usr/include/irrlicht)
	ENDIF()
ENDIF()

IF(BITRL_MQTT)
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC paho-mqttpp3 paho-mqtt3as)
ENDIF()

IF(BITRL_OPENCV)
	TARGET_INCLUDE_DIRECTORIES(bitrllib PUBLIC ${OpenCV_INCLUDE_DIRS})
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC ${OpenCV_LIBS})
ENDIF()

# -----------------------------------------------------------------------------
# Install: the library, its headers, the robot, sensor and world files, and a
# CMake package so that other projects can use find_package(bitrl)
# -----------------------------------------------------------------------------

SET(BITRL_CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/bitrl)

INSTALL(TARGETS bitrllib EXPORT bitrlTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

INSTALL(DIRECTORY src/bitrl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
INSTALL(FILES
	${BITRL_GENERATED_DIR}/bitrl/bitrl_config.h
	${BITRL_GENERATED_DIR}/bitrl/bitrl_version.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bitrl)

INSTALL(DIRECTORY robots sensors worlds DESTINATION ${BITRL_DATA_INSTALL_DIR})

INSTALL(EXPORT bitrlTargets NAMESPACE bitrl:: DESTINATION ${BITRL_CMAKE_INSTALL_DIR})

# What bitrl was built with, for bitrlConfig.cmake
FOREACH(feature CHRONO IRRLICHT MQTT OPENCV)
	IF(BITRL_${feature})
		SET(bitrl_WITH_${feature} ON)
	ELSE()
		SET(bitrl_WITH_${feature} OFF)
	ENDIF()
ENDFOREACH()

CONFIGURE_PACKAGE_CONFIG_FILE(cmake/bitrlConfig.cmake.in
	${PROJECT_BINARY_DIR}/bitrlConfig.cmake
	INSTALL_DESTINATION ${BITRL_CMAKE_INSTALL_DIR}
	PATH_VARS BITRL_DATA_INSTALL_DIR)
WRITE_BASIC_PACKAGE_VERSION_FILE(${PROJECT_BINARY_DIR}/bitrlConfigVersion.cmake
	VERSION ${BITRL_VERSION}
	COMPATIBILITY SameMajorVersion)
INSTALL(FILES
	${PROJECT_BINARY_DIR}/bitrlConfig.cmake
	${PROJECT_BINARY_DIR}/bitrlConfigVersion.cmake
	DESTINATION ${BITRL_CMAKE_INSTALL_DIR})

IF(ENABLE_TESTS)
	# Before the examples, which add smoke tests of their own
	ENABLE_TESTING()
ENDIF()

IF(ENABLE_EXAMPLES)
	# Add the examples
	ADD_SUBDIRECTORY(examples)
ENDIF()

IF(ENABLE_TESTS)
	# Add the tests
	ADD_SUBDIRECTORY(tests)
ENDIF()
