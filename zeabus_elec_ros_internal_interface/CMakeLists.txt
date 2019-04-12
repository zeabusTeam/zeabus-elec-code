cmake_minimum_required(VERSION 3.5)
project(zeabus_elec_ros_internal_interface)

# find dependencies
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  srv/PowerDistSetPowerSwitch.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
