# A project that uses an installed bitsim, the way other projects do.
# ci/check_install.sh builds and runs it. It is not part of the bitsim build.
CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
PROJECT(bitsim_install_consumer LANGUAGES CXX)

FIND_PACKAGE(bitsim REQUIRED)
MESSAGE(STATUS "Found bitsim ${bitsim_VERSION}, Chrono: ${bitsim_WITH_CHRONO}, MQTT: ${bitsim_WITH_MQTT}")

ADD_EXECUTABLE(install_consumer install_consumer.cpp)
TARGET_LINK_LIBRARIES(install_consumer PRIVATE bitsim::bitsim)
