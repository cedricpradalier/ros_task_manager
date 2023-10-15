cmake_minimum_required(VERSION 3.5)
project(task_manager_turtlesim)

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
find_package(task_manager_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(task_manager_move_base)
find_package(task_manager_nav2)

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
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${task_manager_lib_INCLUDE_DIRS}
  ${task_manager_move_base_INCLUDE_DIRS}
  ${task_manager_nav2_INCLUDE_DIRS}
)

SET(ALL_LIBS 
    ${rclcpp_LIBRARIES} 
    ${rclcpp_action_LIBRARIES} 
    ${task_manager_lib_LIBRARIES} 
    ${geometry_msgs_LIBRARIES} 
    ${tf2_geometry_msgs_LIBRARIES} 
    ${task_manager_msgs_LIBRARIES} 
    ${task_manager_move_base_LIBRARIES} 
    ${task_manager_nav2_LIBRARIES} 
    ${std_msgs_LIBRARIES} 
    ${std_srvs_LIBRARIES} 
    ${turtlesim_LIBRARIES}
    ${Boost_THREAD_LIBRARIES} 
    ${CMAKE_DL_LIBS})

file(SHA256 ${PROJECT_SOURCE_DIR}/include/task_manager_turtlesim/TurtleSimEnv.h ENV_CHECK_SUM)
add_compile_definitions(ENV_CHECK_SUM=${ENV_CHECK_SUM})

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

ADD_LIBRARY( turtlesim_TaskSpawn SHARED tasks/TaskSpawnGeneric.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskSpawn ${ALL_LIBS})

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

IF( ${task_manager_move_base_FOUND} )
    ADD_DEFINITIONS(-DTEST_ACTION_CLIENT)
    SET(MoveBase_Targets turtlesim_TaskMoveBase turtlesim_TaskMoveBaseWC)
    ADD_LIBRARY( turtlesim_TaskMoveBase SHARED tasks/TaskMoveBase.cpp)
    TARGET_LINK_LIBRARIES(turtlesim_TaskMoveBase ${ALL_LIBS})

    ADD_LIBRARY( turtlesim_TaskMoveBaseWC SHARED tasks/TaskMoveBaseWC.cpp)
    TARGET_LINK_LIBRARIES(turtlesim_TaskMoveBaseWC ${ALL_LIBS})
endif()
IF( ${task_manager_nav2_FOUND} )
    SET(Nav2_Targets turtlesim_TaskNav2)
    ADD_LIBRARY( turtlesim_TaskNav2 SHARED tasks/TaskNav2.cpp)
    TARGET_LINK_LIBRARIES(turtlesim_TaskNav2 ${ALL_LIBS})
endif()
install(TARGETS
  turtlesim_task_server
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
    turtlesim_TaskClear
    turtlesim_TaskClearGeneric
    turtlesim_TaskClearGenericWC
    turtlesim_TaskSetPen
    turtlesim_TaskSpawn
    turtlesim_TaskFail
    turtlesim_TaskGoTo
    turtlesim_TaskReachAngle
    turtlesim_TaskWaitForROI
    turtlesim_TaskWaitForButton
    ${MoveBase_Targets}
    ${Nav2_Targets}
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS  
    missions/dummy_service.py
    missions/mission.py
    missions/mission_buttons.py
    missions/mission_cond.py
    missions/mission_fail.py
    missions/mission_fast_cond.py
    missions/mission_square.py
    missions/mission_timeout.py
    DESTINATION lib/${PROJECT_NAME} 
)

ament_package()
