CMAKE_MINIMUM_REQUIRED(VERSION 3.20)

# The examples, in the order they are meant to be read: each section uses what the ones before
# it showed. See examples.md.

# BITSIM_ADD_EXAMPLE(<name>
#                   [REQUIRES CHRONO IRRLICHT MQTT OPENCV ...]
#                   [FILES <file> ...]
#                   [SMOKE_TEST] [SMOKE_ARGS <arg> ...])
#
# Builds <name>/<name>.cpp in the current section into the executable <name>, if bitsim was
# configured with everything in REQUIRES. FILES are copied next to the executable. With
# SMOKE_TEST, and ENABLE_TESTS=ON, ctest runs the example with SMOKE_ARGS and expects it to
# exit with 0; ctest -L examples runs only these.
FUNCTION(BITSIM_ADD_EXAMPLE name)
	CMAKE_PARSE_ARGUMENTS(EXAMPLE "SMOKE_TEST" "" "REQUIRES;FILES;SMOKE_ARGS" ${ARGN})
	FOREACH(feature ${EXAMPLE_REQUIRES})
		IF(NOT BITSIM_${feature})
			MESSAGE(STATUS "Example ${name} needs ${feature}; not building it")
			RETURN()
		ENDIF()
	ENDFOREACH()

	SET(dir ${CMAKE_CURRENT_SOURCE_DIR}/${name})
	ADD_EXECUTABLE(${name} ${dir}/${name}.cpp)
	TARGET_INCLUDE_DIRECTORIES(${name} PRIVATE ${dir})
	TARGET_COMPILE_DEFINITIONS(${name} PRIVATE IRRLICHT_NO_XML)
	TARGET_LINK_LIBRARIES(${name} PRIVATE bitsimlib)
	SET_TARGET_PROPERTIES(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
		${CMAKE_CURRENT_BINARY_DIR}/${name})

	FOREACH(file ${EXAMPLE_FILES})
		CONFIGURE_FILE(${dir}/${file} ${CMAKE_CURRENT_BINARY_DIR}/${name}/${file} COPYONLY)
	ENDFOREACH()

	IF(EXAMPLE_SMOKE_TEST AND ENABLE_TESTS)
		ADD_TEST(NAME example_${name} COMMAND ${name} ${EXAMPLE_SMOKE_ARGS}
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${name})
		SET_TESTS_PROPERTIES(example_${name} PROPERTIES LABELS examples TIMEOUT 600)
	ENDIF()
ENDFUNCTION()

ADD_SUBDIRECTORY(01_basics)
ADD_SUBDIRECTORY(02_frames_and_rotations)
ADD_SUBDIRECTORY(03_kinematics)
ADD_SUBDIRECTORY(04_trajectories)
ADD_SUBDIRECTORY(05_motion_planning)
ADD_SUBDIRECTORY(06_sensors_and_actuators)
ADD_SUBDIRECTORY(07_simulation)
ADD_SUBDIRECTORY(08_state_estimation)
ADD_SUBDIRECTORY(09_hardware_in_the_loop)
ADD_SUBDIRECTORY(10_control)
ADD_SUBDIRECTORY(11_parallel)
ADD_SUBDIRECTORY(12_distributed_simulation)
ADD_SUBDIRECTORY(13_co_simulation)
