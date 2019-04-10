cmake_minimum_required(VERSION 3.5)
project(zeabus_elec_ros_power_dist)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

set(srv_files
  srv/PowerDist.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(main
  src/zeabus_elec_ros_power_dist.cpp
  ../zeabus_elec_ftdi_lib/ftdi_impl.cpp
)
ament_target_dependencies(main
  rclcpp
)
target_link_libraries(main
  ftdi1
)

target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../zeabus_elec_ftdi_lib>
    /usr/include/libftdi1
)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(main
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
