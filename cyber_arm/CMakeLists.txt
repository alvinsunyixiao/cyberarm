cmake_minimum_required(VERSION 3.8)
project(cyber_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(cyber_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(cybergear_lib src/cybergear.cc)
target_include_directories(cybergear_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(cybergear_lib PRIVATE Threads::Threads)
ament_target_dependencies(cybergear_lib PUBLIC cyber_msgs)
ament_export_targets(cybergear_libTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(cyber_msgs)

add_executable(test_single_cybergear test/single_motor.cc)
target_link_libraries(test_single_cybergear PUBLIC cybergear_lib)

add_executable(test_multi_cybergear test/multi_motor.cc)
target_link_libraries(test_multi_cybergear PUBLIC cybergear_lib)

add_executable(test_position_mode test/position_mode.cc)
target_link_libraries(test_position_mode PUBLIC cybergear_lib)

add_executable(cyberarm_node src/cyberarm_node.cc)
target_link_libraries(cyberarm_node cybergear_lib)
ament_target_dependencies(cyberarm_node rclcpp sensor_msgs geometry_msgs Eigen3 tf2_eigen)

add_executable(cybergear_node src/cybergear_node.cc)
target_link_libraries(cybergear_node cybergear_lib)
ament_target_dependencies(cybergear_node rclcpp)

# install CPP libraries
install(
  TARGETS cybergear_lib
  EXPORT cybergear_libTarget
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# install shared directories
install(DIRECTORY models launch rviz ml
        DESTINATION share/${PROJECT_NAME})

# install CPP nodes
install(TARGETS cybergear_node cyberarm_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
