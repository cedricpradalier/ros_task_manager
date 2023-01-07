cmake_minimum_required(VERSION 3.5)
project(task_manager_turtlesim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -ggdb)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(task_manager_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(task_manager_move_base REQUIRED)
find_package(turtlesim REQUIRED)
find_package(move_base_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

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
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${move_base_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${task_manager_lib_INCLUDE_DIRS}
  ${task_manager_move_base_INCLUDE_DIRS}
)

SET(ALL_LIBS ${rclcpp_LIBRARIES} ${rclcpp_action_LIBRARIES} ${task_manager_lib_LIBRARIES} ${task_manager_msgs_LIBRARIES} ${task_manager_move_base_LIBRARIES} ${move_base_msgs_LIBRARIES} ${std_msgs_LIBRARIES} ${std_srvs_LIBRARIES} ${turtlesim_LIBRARIES} ${Boost_THREAD_LIBRARIES} ${CMAKE_DL_LIBS})

ADD_EXECUTABLE( turtlesim_task_server src/task_server.cpp src/TurtleSimEnv.cpp) 
ament_target_dependencies(turtlesim_task_server task_manager_lib
    task_manager_msgs geometry_msgs rclcpp std_msgs)
# Necessary target for the tasks to see the symbols of the main
TARGET_LINK_OPTIONS(turtlesim_task_server PRIVATE "-rdynamic")
TARGET_LINK_LIBRARIES(turtlesim_task_server ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskClear SHARED tasks/TaskClear.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskClear ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskClearGeneric SHARED tasks/TaskClearGeneric.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskClearGeneric ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskClearGenericWC SHARED tasks/TaskClearGenericWC.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskClearGenericWC ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskSetPen SHARED tasks/TaskSetPen.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskSetPen ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskFail SHARED tasks/TaskFail.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskFail ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskGoTo SHARED tasks/TaskGoTo.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskGoTo ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskReachAngle SHARED tasks/TaskReachAngle.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskReachAngle ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskWaitForROI SHARED tasks/TaskWaitForROI.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskWaitForROI ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskWaitForButton SHARED tasks/TaskWaitForButton.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskWaitForButton ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskMoveBase SHARED tasks/TaskMoveBase.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskMoveBase ${ALL_LIBS})

ADD_LIBRARY( turtlesim_TaskMoveBaseWC SHARED tasks/TaskMoveBaseWC.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskMoveBaseWC ${ALL_LIBS})

install(TARGETS
  turtlesim_task_server
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
    turtlesim_TaskClear
    turtlesim_TaskClearGeneric
    turtlesim_TaskClearGenericWC
    turtlesim_TaskSetPen
    turtlesim_TaskFail
    turtlesim_TaskGoTo
    turtlesim_TaskReachAngle
    turtlesim_TaskWaitForROI
    turtlesim_TaskWaitForButton
    turtlesim_TaskMoveBase
    turtlesim_TaskMoveBaseWC
    DESTINATION lib)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
