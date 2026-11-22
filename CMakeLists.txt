CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")

# version.txt is the only place the version is kept. release-please updates
# it when it makes a release, so do not change it by hand
FILE(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" BITSIM_VERSION LIMIT_COUNT 1)
IF(NOT BITSIM_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
	MESSAGE(FATAL_ERROR "version.txt must hold a version such as 1.2.3, not '${BITSIM_VERSION}'")
ENDIF()
SET(BITSIM_VERSION_MAJOR ${CMAKE_MATCH_1})
SET(BITSIM_VERSION_MINOR ${CMAKE_MATCH_2})
SET(BITSIM_VERSION_PATCH ${CMAKE_MATCH_3})
SET_PROPERTY(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
MESSAGE(STATUS "bitsimlib Version ${BITSIM_VERSION}")

# create a CMake project
PROJECT(bitsim VERSION ${BITSIM_VERSION} LANGUAGES CXX)

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
OPTION(BITSIM_LOG "Log various messages from the library" ON)
OPTION(BITSIM_IRRLICHT "Use Irrlicht for visualization. Requires ENABLE_CHRONO" ON)
OPTION(BITSIM_WERROR "Treat compiler warnings as errors in Debug builds" OFF)
OPTION(BITSIM_SANITIZE "Build bitsim, its tests and examples with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

IF(NOT ENABLE_CHRONO)
	SET(BITSIM_IRRLICHT OFF)
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type" FORCE)
ENDIF()

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(BITSIM_DEBUG ON)
ELSE()
	SET(BITSIM_DEBUG OFF)
ENDIF()

INCLUDE(GNUInstallDirs)
INCLUDE(CMakePackageConfigHelpers)

# Without -DCMAKE_INSTALL_PREFIX bitsim installs into the source tree, as it
# always did: install/opt for a Release build and install/dbg for a Debug build
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	IF(BITSIM_DEBUG)
		SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install/dbg" CACHE PATH "Install prefix" FORCE)
	ELSE()
		SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install/opt" CACHE PATH "Install prefix" FORCE)
	ENDIF()
ENDIF()

SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The warnings for bitsim's own code: the library and the tests
FUNCTION(BITSIM_SET_WARNINGS target)
	TARGET_COMPILE_OPTIONS(${target} PRIVATE -Wall -Wpedantic)
	IF(BITSIM_WERROR AND BITSIM_DEBUG)
		TARGET_COMPILE_OPTIONS(${target} PRIVATE -Werror)
	ENDIF()
ENDFUNCTION()

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find packages that we need
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Boost 1.74.0 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(spdlog REQUIRED)

IF(ENABLE_MQTT)
	FIND_PACKAGE(PahoMqttCpp REQUIRED)
	SET(BITSIM_MQTT ON)
ENDIF()

IF(ENABLE_OPENCV)
	FIND_PACKAGE(OpenCV REQUIRED)
	SET(BITSIM_OPENCV ON)
ENDIF()

IF(ENABLE_TESTS)
	FIND_PACKAGE(GTest REQUIRED)
ENDIF()

IF(ENABLE_CHRONO)
	IF(BITSIM_IRRLICHT)
		FIND_PACKAGE(Chrono REQUIRED COMPONENTS Irrlicht)
	ELSE()
		FIND_PACKAGE(Chrono REQUIRED)
	ENDIF()
	SET(BITSIM_CHRONO ON)
	SET(BITSIM_CHRONO_TARGETS ${CHRONO_TARGETS})
ENDIF()

# -----------------------------------------------------------------------------
# Generated headers. They go into the build directory
# -----------------------------------------------------------------------------

# Earlier builds wrote these into src/. Remove them so they cannot shadow the
# generated ones
FILE(REMOVE ${PROJECT_SOURCE_DIR}/src/bitsim/bitsim_config.h ${PROJECT_SOURCE_DIR}/src/bitsim/bitsim_version.h)

SET(BITSIM_GENERATED_DIR ${PROJECT_BINARY_DIR}/generated)

# The robot, sensor and world files. bitsim::consts::data_dir() picks one of
# these at run time
SET(BITSIM_DATA_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/bitsim)
SET(BITSIM_SOURCE_DATA_DIR ${PROJECT_SOURCE_DIR})
SET(BITSIM_INSTALL_DATA_DIR ${CMAKE_INSTALL_FULL_DATADIR}/bitsim)

CONFIGURE_FILE(config.h.in ${BITSIM_GENERATED_DIR}/bitsim/bitsim_config.h @ONLY)
CONFIGURE_FILE(version.h.in ${BITSIM_GENERATED_DIR}/bitsim/bitsim_version.h @ONLY)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Boost include: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Install dst: ${CMAKE_INSTALL_PREFIX}")

# -----------------------------------------------------------------------------
# The library
# -----------------------------------------------------------------------------

FILE(GLOB SRCS CONFIGURE_DEPENDS
		src/bitsim/*.cpp
		src/bitsim/actuators/*.cpp
		src/bitsim/actuators/backends/*.cpp
		src/bitsim/boards/*.cpp
		src/bitsim/boards/arduino/*.cpp
		src/bitsim/control/*.cpp
		src/bitsim/dynamics/*.cpp
		src/bitsim/estimation/*.cpp
		src/bitsim/geometry/*.cpp
		src/bitsim/kinematics/*.cpp
		src/bitsim/network/*.cpp
		src/bitsim/planning/*.cpp
		src/bitsim/rigid_bodies/*.cpp
		src/bitsim/rigid_bodies/chrono_robots/*.cpp
		src/bitsim/rigid_bodies/chrono_robots/impl/*.cpp
		src/bitsim/sensors/*.cpp
		src/bitsim/sensors/messages/*.cpp
		src/bitsim/sensors/backends/*.cpp
		src/bitsim/systems/*.cpp
		src/bitsim/trajectory/*.cpp
		src/bitsim/utils/*.cpp
		src/bitsim/utils/io/*.cpp
		src/bitsim/utils/concurrency/*.cpp
		src/bitsim/maths/statistics/distributions/*.cpp
		src/bitsim/maths/optimization/*.cpp
		src/bitsim/utils/render/*.cpp
   )

ADD_LIBRARY(bitsimlib SHARED ${SRCS})

# The files that use Eigen's sparse matrices keep the Eigen code they instantiate inside bitsim:
# shared, it would replace the same code in Chrono, which may be compiled with other vector
# instructions, and crash it. Their classes are exported with BITSIM_EXPORT
SET_SOURCE_FILES_PROPERTIES(
	src/bitsim/maths/optimization/qp_solver.cpp
	src/bitsim/control/linear_mpc.cpp
	PROPERTIES COMPILE_OPTIONS "-fvisibility=hidden;-fvisibility-inlines-hidden")
ADD_LIBRARY(bitsim::bitsim ALIAS bitsimlib)

SET_TARGET_PROPERTIES(bitsimlib PROPERTIES
	LINKER_LANGUAGE CXX
	EXPORT_NAME bitsim
	VERSION ${BITSIM_VERSION}
	SOVERSION ${BITSIM_VERSION_MAJOR}
	# The installed library finds the libraries it links to that are not in
	# a system directory, Chrono for example, where they were at build time
	INSTALL_RPATH_USE_LINK_PATH TRUE)

BITSIM_SET_WARNINGS(bitsimlib)

IF(BITSIM_SANITIZE)
	# Only for the build tree: an installed bitsim does not force the
	# sanitizers on the projects that use it
	SET(BITSIM_SANITIZER_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
	TARGET_COMPILE_OPTIONS(bitsimlib PUBLIC $<BUILD_INTERFACE:${BITSIM_SANITIZER_FLAGS}>)
	TARGET_LINK_OPTIONS(bitsimlib PUBLIC $<BUILD_INTERFACE:${BITSIM_SANITIZER_FLAGS}>)
ENDIF()

# Everything that links against bitsimlib inherits these
TARGET_COMPILE_FEATURES(bitsimlib PUBLIC cxx_std_20)
TARGET_INCLUDE_DIRECTORIES(bitsimlib PUBLIC
	$<BUILD_INTERFACE:${BITSIM_GENERATED_DIR}>
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
TARGET_LINK_LIBRARIES(bitsimlib PUBLIC Boost::headers Threads::Threads Eigen3::Eigen)

# Only the implementation of bitsim/utils/logger.h uses spdlog, so the projects
# that use bitsim do not need it to compile
TARGET_LINK_LIBRARIES(bitsimlib PRIVATE spdlog::spdlog)

IF(BITSIM_CHRONO)
	TARGET_LINK_LIBRARIES(bitsimlib PUBLIC Chrono::Chrono_core)
	IF(BITSIM_IRRLICHT)
		TARGET_LINK_LIBRARIES(bitsimlib PUBLIC Chrono::Chrono_irrlicht)
		# Chrono's Irrlicht target does not carry the Irrlicht headers
		TARGET_INCLUDE_DIRECTORIES(bitsimlib PUBLIC /usr/include/irrlicht)
	ENDIF()
ENDIF()

IF(BITSIM_MQTT)
	TARGET_LINK_LIBRARIES(bitsimlib PUBLIC paho-mqttpp3 paho-mqtt3as)
ENDIF()

IF(BITSIM_OPENCV)
	TARGET_INCLUDE_DIRECTORIES(bitsimlib PUBLIC ${OpenCV_INCLUDE_DIRS})
	TARGET_LINK_LIBRARIES(bitsimlib PUBLIC ${OpenCV_LIBS})
ENDIF()

# -----------------------------------------------------------------------------
# Install: the library, its headers, the robot, sensor and world files, and a
# CMake package so that other projects can use find_package(bitsim)
# -----------------------------------------------------------------------------

SET(BITSIM_CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/bitsim)

INSTALL(TARGETS bitsimlib EXPORT bitsimTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

INSTALL(DIRECTORY src/bitsim DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
INSTALL(FILES
	${BITSIM_GENERATED_DIR}/bitsim/bitsim_config.h
	${BITSIM_GENERATED_DIR}/bitsim/bitsim_version.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bitsim)

INSTALL(DIRECTORY robots sensors DESTINATION ${BITSIM_DATA_INSTALL_DIR})

INSTALL(EXPORT bitsimTargets NAMESPACE bitsim:: DESTINATION ${BITSIM_CMAKE_INSTALL_DIR})

# What bitsim was built with, for bitsimConfig.cmake
FOREACH(feature CHRONO IRRLICHT MQTT OPENCV)
	IF(BITSIM_${feature})
		SET(bitsim_WITH_${feature} ON)
	ELSE()
		SET(bitsim_WITH_${feature} OFF)
	ENDIF()
ENDFOREACH()

CONFIGURE_PACKAGE_CONFIG_FILE(cmake/bitsimConfig.cmake.in
	${PROJECT_BINARY_DIR}/bitsimConfig.cmake
	INSTALL_DESTINATION ${BITSIM_CMAKE_INSTALL_DIR}
	PATH_VARS BITSIM_DATA_INSTALL_DIR)
WRITE_BASIC_PACKAGE_VERSION_FILE(${PROJECT_BINARY_DIR}/bitsimConfigVersion.cmake
	VERSION ${BITSIM_VERSION}
	COMPATIBILITY SameMajorVersion)
INSTALL(FILES
	${PROJECT_BINARY_DIR}/bitsimConfig.cmake
	${PROJECT_BINARY_DIR}/bitsimConfigVersion.cmake
	DESTINATION ${BITSIM_CMAKE_INSTALL_DIR})

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
