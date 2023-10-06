cmake_minimum_required(VERSION 3.5)
project(task_manager_move_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(task_manager_msgs REQUIRED)
find_package(move_base_msgs REQUIRED)
find_package(tf2 REQUIRED)

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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${move_base_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

install(
  DIRECTORY include/task_manager_move_base
  DESTINATION include
)

ament_export_dependencies(
    rclcpp
    rclcpp_action
    task_manager_msgs
    move_base_msgs
    task_manager_lib
    tf2
)
ament_export_include_directories(include)


ament_package()
