cmake_minimum_required(VERSION 3.8)
project(lora_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(slam_toolbox REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_executable(lora_goal_receiver src/lora_goal_receiver.cpp)
ament_target_dependencies(lora_goal_receiver rclcpp geometry_msgs tf2 slam_toolbox nav2_msgs rclcpp_action)
target_include_directories(lora_goal_receiver PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(lora_goal_receiver "${SERIAL_LDFLAGS}")

install(TARGETS
  lora_goal_receiver
  DESTINATION lib/${PROJECT_NAME}
)


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
