cmake_minimum_required(VERSION 3.8)
project(rplidar_filtered_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(rplidar_subscriber src/rplidar_subscriber.cpp)

ament_target_dependencies(rplidar_subscriber
  rclcpp
  sensor_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rplidar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()