cmake_minimum_required(VERSION 3.8)
project(handpoints_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- generate custom messages ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandKeypoints.msg"
  DEPENDENCIES geometry_msgs
)

# --- add executable ---
add_executable(handpoints_pub src/handpoints_pub.cpp)
ament_target_dependencies(handpoints_pub rclcpp geometry_msgs)

# Use recommended rosidl link method instead of deprecated rosidl_target_interfaces
rosidl_get_typesupport_target(rosidl_typesupport_cpp_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(handpoints_pub ${rosidl_typesupport_cpp_target})

# --- install executable ---
install(TARGETS
  handpoints_pub
  DESTINATION lib/${PROJECT_NAME}
)

# --- install data directory ---
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

# --- export dependencies ---
ament_export_dependencies(rosidl_default_runtime)

ament_package()
