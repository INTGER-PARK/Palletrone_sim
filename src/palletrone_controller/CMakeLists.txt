cmake_minimum_required(VERSION 3.8)
project(palletrone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(palletrone_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# wrench_controller
add_executable(wrench_controller src/wrench_controller.cpp)
ament_target_dependencies(wrench_controller rclcpp palletrone_interfaces)
target_link_libraries(wrench_controller Eigen3::Eigen)

# allocator_controller
add_executable(allocator_controller src/allocator_controller.cpp)
ament_target_dependencies(allocator_controller rclcpp palletrone_interfaces)
target_link_libraries(allocator_controller Eigen3::Eigen)

# wrench_observer
add_executable(wrench_observer src/wrench_observer.cpp)
ament_target_dependencies(wrench_observer rclcpp palletrone_interfaces)
target_link_libraries(wrench_observer Eigen3::Eigen)

# ekf_state_estimator
add_executable(ekf_state_estimator src/ekf_state_estimator.cpp)
ament_target_dependencies(ekf_state_estimator rclcpp palletrone_interfaces std_msgs)
target_link_libraries(ekf_state_estimator Eigen3::Eigen)

install(TARGETS
  wrench_controller
  allocator_controller
  wrench_observer
  ekf_state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
