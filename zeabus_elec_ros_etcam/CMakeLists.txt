cmake_minimum_required(VERSION 2.8.3)
project(zeabus_elec_ros_etcam)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    zeabus_library_cpp
)

add_service_files(FILES
    CommandSetThrusterThrottle.srv
)

add_message_files(FILES
    Telemetry.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    zeabus_library_cpp
)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ../libetcam/include
)

add_executable(zeabus_elec_ros_etcam 
    src/zeabus_elec_ros_etcam.cpp 
    ../libetcam/src/libetcam.cpp
)
add_dependencies(zeabus_elec_ros_etcam
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(zeabus_elec_ros_etcam
    ${catkin_LIBRARIES}
    ${zeabus_library_cpp_LIBRARIES}
)
