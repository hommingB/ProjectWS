cmake_minimum_required(VERSION 3.8)
project(robot_diffdrive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib          REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)

# ── Shared library (the plugin .so) ──────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  hardware/esp32_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# ── pluginlib export ──────────────────────────────────────────────────────────
pluginlib_export_plugin_description_file(hardware_interface esp32_hardware.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY hardware/include/
  DESTINATION include
)

install(FILES esp32_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# install launch files and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── ament package ─────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()