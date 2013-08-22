cmake_minimum_required(VERSION 2.8.3)
project(task_manager_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp task_manager_lib task_manager_msgs message_generation )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  cfg/TaskLong.cfg
  cfg/TaskTest.cfg
)



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
#  INCLUDE_DIRS include
#  LIBRARIES task_manager_test
  CATKIN_DEPENDS dynamic_reconfigure roscpp task_manager_lib task_manager_msgs message_runtime
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
# add_library(task_manager_test
#   src/${PROJECT_NAME}/task_manager_test.cpp
# )

## Declare a cpp executable
# add_executable(task_manager_test_node src/task_manager_test_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(task_manager_test_node task_manager_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(task_manager_test_node
#   ${catkin_LIBRARIES}
# )


# does not compile anymore

# ADD_LIBRARY( ${PROJECT_NAME}_TaskTest SHARED src/TaskTest.cpp)
# TARGET_LINK_LIBRARIES(${PROJECT_NAME}_TaskTest ${catkin_LIBRARIES} dl)
# add_dependencies( ${PROJECT_NAME}_TaskTest ${${PROJECT_NAME}_EXPORTED_TARGETS})
# 
# 
# ADD_LIBRARY( ${PROJECT_NAME}_TaskFail SHARED src/TaskFail.cpp)
# TARGET_LINK_LIBRARIES(${PROJECT_NAME}_TaskFail ${catkin_LIBRARIES} dl)
# add_dependencies( ${PROJECT_NAME}_TaskFail ${${PROJECT_NAME}_EXPORTED_TARGETS})
# 
# ADD_LIBRARY( ${PROJECT_NAME}_TaskLong SHARED src/TaskLong.cpp)
# TARGET_LINK_LIBRARIES(${PROJECT_NAME}_TaskLong ${catkin_LIBRARIES} dl)
# add_dependencies( ${PROJECT_NAME}_TaskLong ${${PROJECT_NAME}_EXPORTED_TARGETS})
# 
# ADD_LIBRARY( ${PROJECT_NAME}_TaskIdle STATIC src/TaskIdle.cpp)
# TARGET_LINK_LIBRARIES(${PROJECT_NAME}_TaskIdle ${catkin_LIBRARIES} dl)
# add_dependencies( ${PROJECT_NAME}_TaskIdle ${${PROJECT_NAME}_EXPORTED_TARGETS})
# 
# 
# ADD_EXECUTABLE( testTasks src/testTasks.cpp)
# TARGET_LINK_LIBRARIES( testTasks TaskIdle ${catkin_LIBRARIES} dl)
# 	
# 
# ADD_EXECUTABLE( testTS1a src/testTS1a.cpp)
# TARGET_LINK_LIBRARIES( testTS1a TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS1b src/testTS1b.cpp)
# TARGET_LINK_LIBRARIES( testTS1b TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS1c src/testTS1c.cpp)
# TARGET_LINK_LIBRARIES( testTS1c TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS2a src/testTS2a.cpp) 
# TARGET_LINK_LIBRARIES( testTS2a TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS2b src/testTS2b.cpp) 
# TARGET_LINK_LIBRARIES( testTS2b TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS2c src/testTS2c.cpp) 
# TARGET_LINK_LIBRARIES( testTS2c TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS2d src/testTS2d.cpp) 
# TARGET_LINK_LIBRARIES( testTS2d TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 
# ADD_EXECUTABLE( testTS2e src/testTS2e.cpp) 
# TARGET_LINK_LIBRARIES( testTS2e TaskIdle ${catkin_LIBRARIES} dl )
# 	
# ADD_EXECUTABLE( testTS2f src/testTS2f.cpp) 
# TARGET_LINK_LIBRARIES( testTS2f TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 	
# ADD_EXECUTABLE( testServer src/testServer.cpp) 
# TARGET_LINK_LIBRARIES( testServer TaskIdle ${catkin_LIBRARIES} dl )
# 	
# 	
# ADD_EXECUTABLE( testClienta src/testClienta.cpp) 
# TARGET_LINK_LIBRARIES( testClienta ${catkin_LIBRARIES})
# 	
# 	
# ADD_EXECUTABLE( testClientb src/testClientb.cpp) 
# TARGET_LINK_LIBRARIES( testClientb ${catkin_LIBRARIES})
# 	
# 	
# ADD_EXECUTABLE( testClientc src/testClientc.cpp) 
# TARGET_LINK_LIBRARIES( testClientc ${catkin_LIBRARIES})
# 	
# 	
# ADD_EXECUTABLE( testClientd src/testClientd.cpp) 
# TARGET_LINK_LIBRARIES( testClientd ${catkin_LIBRARIES})
# 	
# 	
# ADD_EXECUTABLE( testCliente src/testCliente.cpp) 
# TARGET_LINK_LIBRARIES( testCliente ${catkin_LIBRARIES})
# 	


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
# install(TARGETS task_manager_test task_manager_test_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_manager_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
