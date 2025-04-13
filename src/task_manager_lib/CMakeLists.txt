cmake_minimum_required(VERSION 3.5)
project(task_manager_lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -ggdb)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(task_manager_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
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
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
)

add_compile_definitions(ENV_CHECK_SUM=task_manager_lib)

add_library(task_manager_lib SHARED
    src/DynamicTask.cpp
    src/TaskConfig.cpp
    src/TaskInstance.cpp
    src/TaskDefinition.cpp
    src/TaskScheduler.cpp
    src/TaskIdleDefault.cpp
    src/TaskWaitDefault.cpp
    src/TaskSystem.cpp
    src/TaskServerDefault.cpp
)



target_link_libraries(task_manager_lib ${rclcpp_LIBRARIES} ${rclcpp_action_LIBRARIES} ${task_manager_msgs_LIBRARIES} ${std_msgs_LIBRARIES} ${std_srvs_LIBRARIES} ${Boost_THREAD_LIBRARIES} ${CMAKE_DL_LIBS})
 
#ament_export_targets(task_manager_lib HAS_LIBRARY_TARGET) 

install(TARGETS
  task_manager_lib
  DESTINATION lib)

install(
  DIRECTORY include/task_manager_lib
  DESTINATION include
)

ament_export_dependencies(
    Boost
    rclcpp
    rclcpp_action
    rcl_interfaces
    task_manager_msgs
    std_msgs
    std_srvs
)
ament_export_include_directories(include)
ament_export_libraries(task_manager_lib)

ament_package()
