cmake_minimum_required(VERSION 2.8.3)
project(task_manager_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp roslib rospy std_srvs task_manager_msgs  message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
add_service_files(
  FILES
  ExeComplexMission.srv
  ExeTaskSequence.srv
  GetAllTaskStatus.srv
  GetHistory.srv
  GetTaskListLight.srv
  GetTaskList.srv
  ListMissions.srv
  SaveBasicMission.srv
  SaveComplexMission.srv
  StartTask.srv
  StopComplexMission.srv
  StopTask.srv
)

generate_dynamic_reconfigure_options(
    cfg/MinimalTask.cfg
    cfg/TaskWait.cfg
)

# make sure configure headers are built before any node using them


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  task_manager_msgs
)

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
  INCLUDE_DIRS include
  LIBRARIES task_manager_lib
  CATKIN_DEPENDS dynamic_reconfigure roscpp roslib rospy std_srvs task_manager_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(task_manager_lib
    src/DynamicTask.cpp
    src/TaskClient.cpp
    src/TaskDefinition.cpp
    src/TaskScheduler.cpp
    src/SequenceTask.cpp
    src/TaskIdleDefault.cpp
    src/TaskWaitDefault.cpp
    src/TaskServerInterface.cpp
    src/TaskHistory.cpp
)
add_dependencies(task_manager_lib 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    task_manager_lib_generate_message_cpp
    task_manager_msgs_generate_message_cpp)
target_link_libraries(task_manager_lib
  ${catkin_LIBRARIES} dl
)

## Declare a cpp executable
# add_executable(task_manager_lib_node src/task_manager_lib_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(task_manager_lib_node task_manager_lib_generate_messages_cpp)

## Specify libraries to link a library or executable target against

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  node/console
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS task_manager_lib 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  node/console.py
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_manager_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
