cmake_minimum_required(VERSION 3.5)
project(task_manager_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetAllTaskStatus.srv"
    "srv/StartTask.srv"
    "srv/StopTask.srv"
    "srv/GetTaskList.srv"
    "msg/TaskDescription.msg"
    "msg/TaskParameter.msg"
    "msg/TaskStatus.msg"
    "msg/TaskConfig.msg"
    "msg/SyncStatus.msg"
    "msg/EncapsulatedMessage.msg"
  DEPENDENCIES std_msgs rcl_interfaces # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)



include_directories(include)

add_executable(task_manager_msgs_encapsulate src/test_encapsulate.cpp)
ament_target_dependencies(task_manager_msgs_encapsulate rclcpp std_msgs)
rosidl_target_interfaces(task_manager_msgs_encapsulate ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Probably for humble
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(task_manager_msgs_encapsulate "${cpp_typesupport_target}")

install(TARGETS
  task_manager_msgs_encapsulate
  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(
    rclcpp
    rcl_interfaces
    std_msgs
    rosidl_typesupport_cpp
)
ament_package()
