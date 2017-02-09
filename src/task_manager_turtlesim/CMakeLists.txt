cmake_minimum_required(VERSION 2.8.3)
project(task_manager_turtlesim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs task_manager_lib task_manager_action task_manager_msgs turtlesim )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_dynamic_reconfigure_options(
    cfg/TaskFail.cfg
    cfg/TaskGoTo.cfg
    cfg/TaskReachAngle.cfg
    cfg/TaskSetPen.cfg
    cfg/TaskWaitForButton.cfg
    cfg/TaskWaitForROI.cfg
)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs   task_manager_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_manager_turtlesim
  CATKIN_DEPENDS roscpp std_msgs task_manager_lib task_manager_msgs turtlesim 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(task_manager_turtlesim
#   src/${PROJECT_NAME}/task_manager_turtlesim.cpp
# )

## Declare a cpp executable
# add_executable(task_manager_turtlesim_node src/task_manager_turtlesim_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(task_manager_turtlesim_node task_manager_turtlesim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(task_manager_turtlesim_node
#   ${catkin_LIBRARIES}
# )

ADD_EXECUTABLE( turtlesim_task_server src/task_server.cpp src/TurtleSimEnv.cpp) 
TARGET_LINK_LIBRARIES(turtlesim_task_server ${catkin_LIBRARIES} dl )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tasks)

ADD_LIBRARY( turtlesim_TaskGoTo SHARED tasks/TaskGoTo.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskGoTo ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskGoTo ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskReachAngle SHARED tasks/TaskReachAngle.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskReachAngle ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskReachAngle ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskSetPen SHARED tasks/TaskSetPen.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskSetPen ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskSetPen ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskClear SHARED tasks/TaskClear.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskClear ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskClear ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskWaitForROI SHARED tasks/TaskWaitForROI.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskWaitForROI ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskWaitForROI ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskWaitForButton SHARED tasks/TaskWaitForButton.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskWaitForButton ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskWaitForButton ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskFail SHARED tasks/TaskFail.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskFail ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskFail ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskMoveBase SHARED tasks/TaskMoveBase.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskMoveBase ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskMoveBase ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( turtlesim_TaskMoveGoal SHARED tasks/TaskMoveGoal.cpp)
TARGET_LINK_LIBRARIES(turtlesim_TaskMoveGoal ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(turtlesim_TaskMoveGoal ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS task_manager_turtlesim task_manager_turtlesim_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_manager_turtlesim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
