cmake_minimum_required(VERSION 3.8)
project(robot_commander_two)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Custom messages ───────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServiceRequest.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

set(DEPS
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  action_msgs
  rosidl_default_runtime
)

# ── WaypointCommander ─────────────────────────────────────────────────────────
add_executable(wp_commander
  src/wp_commander.cpp
)
target_include_directories(wp_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wp_commander ${DEPS})

# ── ModeManager ───────────────────────────────────────────────────────────────
add_executable(mode_manager
  src/mode_manager.cpp
  src/request_fsm_types.cpp
)
target_include_directories(mode_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
ament_target_dependencies(mode_manager ${DEPS})

# Link generated message types to mode_manager
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mode_manager "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS wp_commander mode_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
