cmake_minimum_required(VERSION 2.8.3)
project(zeabus_elec_ros)

## Compile as C++14
add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    zeabus_utility
    zeabus_library_cpp
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
pkg_check_modules(FTDI REQUIRED
    libftdi1
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    MessageNodeStatus.msg
    MessageHardwareError.msg
    MessageAction.msg
    MessageBarometerValue.msg
    MessageIOPinState.msg
    MessageTelemetryValue.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    ServiceSetPowerSwitch.srv
    ServiceSetIOPinState.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    zeabus_utility
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES zeabus_elec_ros
    CATKIN_DEPENDS
    roscpp
    message_runtime
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FTDI_INCLUDE_DIRS}
  include
  ../zeabus_elec_ftdi_lib
  ../libetcam/include/
)

link_directories(
    ../libetcam
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable( power_distributor_node
  src/power_distributor.cpp
  src/logger.cpp
  ../zeabus_elec_ftdi_lib/ftdi_impl.cpp
)

add_executable( peripheral_bridge_node
  src/peripheral_bridge.cpp
  src/logger.cpp
  ../zeabus_elec_ftdi_lib/ftdi_impl.cpp
)

add_executable( etcam_node
  src/etcam.cpp
  src/logger.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies( power_distributor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies( peripheral_bridge_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies( etcam_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries( power_distributor_node
  ${catkin_LIBRARIES}
  ftdi1
)

target_link_libraries( peripheral_bridge_node
  ${catkin_LIBRARIES}
  ftdi1
)

target_link_libraries( etcam_node
  ${catkin_LIBRARIES}
  ${zeabus_library_cpp_LIBRARIES}
  etcam
)
