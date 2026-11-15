# A project that uses an installed bitrl, the way other projects do.
# ci/check_install.sh builds and runs it. It is not part of the bitrl build.
CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
PROJECT(bitrl_install_consumer LANGUAGES CXX)

FIND_PACKAGE(bitrl 2 REQUIRED)
MESSAGE(STATUS "Found bitrl ${bitrl_VERSION}, Chrono: ${bitrl_WITH_CHRONO}, MQTT: ${bitrl_WITH_MQTT}")

ADD_EXECUTABLE(install_consumer install_consumer.cpp)
TARGET_LINK_LIBRARIES(install_consumer PRIVATE bitrl::bitrl)
