cmake_minimum_required(VERSION 3.5)
project(task_manager_sync)

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
find_package(task_manager_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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
  ${std_msgs_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${task_manager_lib_INCLUDE_DIRS}
)

add_compile_definitions(ENV_CHECK_SUM=task_manager_sync)

add_library(task_manager_sync_lib SHARED
    src/TaskEnvironmentSync.cpp
    src/TaskServerSync.cpp
    src/TaskSetStatusSync.cpp
    src/TaskWaitForStatusSync.cpp
)

target_link_libraries(task_manager_sync_lib ${rclcpp_LIBRARIES} ${task_manager_msgs_LIBRARIES} ${task_manager_lib_LIBRARIES} ${std_msgs_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(test_server_sync src/task_server.cpp)
target_link_libraries(test_server_sync task_manager_sync_lib)

install(TARGETS
  task_manager_sync_lib
  DESTINATION lib)

install(
  DIRECTORY include/task_manager_sync
  DESTINATION include
)

# install(TARGETS
#   test_server_sync
#   DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)


ament_export_dependencies(
    rclcpp
    std_msgs
    rosidl_typesupport_cpp
    task_manager_msgs
    task_manager_lib
)
ament_export_include_directories(include)
ament_export_libraries(task_manager_sync_lib)

ament_package()
