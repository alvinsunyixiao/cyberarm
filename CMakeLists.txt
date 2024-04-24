cmake_minimum_required(VERSION 3.8)
project(cyberarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(cybergear_lib src/cybergear.cc)
target_include_directories(cybergear_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_targets(cybergear_libTarget HAS_LIBRARY_TARGET)

add_executable(test_single_cybergear test/single_motor.cc)
target_link_libraries(test_single_cybergear PUBLIC cybergear_lib)

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
install(DIRECTORY models launch
        DESTINATION share/${PROJECT_NAME})

# install CPP nodes
install(TARGETS test_single_cybergear
        DESTINATION lib/${PROJECT_NAME})

# install Python nodes
install(PROGRAMS
    scripts/control.py
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
