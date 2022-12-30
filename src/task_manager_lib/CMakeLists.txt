cmake_minimum_required(VERSION 3.5)
project(task_manager_lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(task_manager_msgs REQUIRED)
# find_package(dlopen REQUIRED)

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
  ${rclcpp_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
)

add_library(task_manager_lib
    src/DynamicTask.cpp
    src/TaskDefinition.cpp
    src/TaskScheduler.cpp
    src/TaskIdleDefault.cpp
    src/TaskWaitDefault.cpp
    # src/ServiceStorage.cpp
    # The files below are not used in ROS2 version for now.
    # src/SequenceTask.cpp
    # src/TaskServerInterface.cpp
    # src/TaskHistory.cpp
    # src/TaskClient.cpp
)



target_link_libraries(task_manager_lib ${rclcpp_LIBRARIES} ${task_manager_msgs_LIBRARIES} ${CMAKE_DL_LIBS})
 

install(TARGETS
  task_manager_lib
  DESTINATION lib/${PROJECT_NAME})



ament_package()
