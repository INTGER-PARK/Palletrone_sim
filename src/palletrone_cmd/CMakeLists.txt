cmake_minimum_required(VERSION 3.8)
project(palletrone_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(palletrone_interfaces REQUIRED)

add_executable(position_cmd src/position_cmd.cpp)
ament_target_dependencies(position_cmd rclcpp palletrone_interfaces)

add_executable(attitude_sweep_cmd src/attitude_sweep_cmd.cpp)
ament_target_dependencies(attitude_sweep_cmd
  rclcpp
  palletrone_interfaces
)

add_executable(external_wrench_cmd src/external_wrench_cmd.cpp)
ament_target_dependencies(external_wrench_cmd
  rclcpp
  palletrone_interfaces
)

install(TARGETS
  attitude_sweep_cmd
  external_wrench_cmd
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS position_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
