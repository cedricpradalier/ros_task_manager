cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


ROSBUILD_ADD_LIBRARY( TaskTest SHARED src/TaskTest.cpp)
TARGET_LINK_LIBRARIES(TaskTest dl)

ROSBUILD_ADD_LIBRARY( TaskFail SHARED src/TaskFail.cpp)
TARGET_LINK_LIBRARIES(TaskFail dl)

ROSBUILD_ADD_LIBRARY( TaskLong SHARED src/TaskLong.cpp)
TARGET_LINK_LIBRARIES(TaskLong dl)

ROSBUILD_ADD_LIBRARY( TaskIdle STATIC src/TaskIdle.cpp)
TARGET_LINK_LIBRARIES(TaskIdle dl)


ROSBUILD_ADD_EXECUTABLE( testTasks src/testTasks.cpp)
TARGET_LINK_LIBRARIES( testTasks TaskIdle dl)
	

ROSBUILD_ADD_EXECUTABLE( testTS1a src/testTS1a.cpp)
TARGET_LINK_LIBRARIES( testTS1a TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS1b src/testTS1b.cpp)
TARGET_LINK_LIBRARIES( testTS1b TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS1c src/testTS1c.cpp)
TARGET_LINK_LIBRARIES( testTS1c TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS2a src/testTS2a.cpp) 
TARGET_LINK_LIBRARIES( testTS2a TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS2b src/testTS2b.cpp) 
TARGET_LINK_LIBRARIES( testTS2b TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS2c src/testTS2c.cpp) 
TARGET_LINK_LIBRARIES( testTS2c TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS2d src/testTS2d.cpp) 
TARGET_LINK_LIBRARIES( testTS2d TaskIdle dl )
	

ROSBUILD_ADD_EXECUTABLE( testTS2e src/testTS2e.cpp) 
TARGET_LINK_LIBRARIES( testTS2e TaskIdle dl )
	
ROSBUILD_ADD_EXECUTABLE( testTS2f src/testTS2f.cpp) 
TARGET_LINK_LIBRARIES( testTS2f TaskIdle dl )
	
	
ROSBUILD_ADD_EXECUTABLE( testServer src/testServer.cpp) 
TARGET_LINK_LIBRARIES( testServer TaskIdle dl )
	
	
ROSBUILD_ADD_EXECUTABLE( testClienta src/testClienta.cpp) 
TARGET_LINK_LIBRARIES( testClienta )
	
	
ROSBUILD_ADD_EXECUTABLE( testClientb src/testClientb.cpp) 
TARGET_LINK_LIBRARIES( testClientb )
	
	
ROSBUILD_ADD_EXECUTABLE( testClientc src/testClientc.cpp) 
TARGET_LINK_LIBRARIES( testClientc )
	
	
ROSBUILD_ADD_EXECUTABLE( testClientd src/testClientd.cpp) 
TARGET_LINK_LIBRARIES( testClientd )
	
	
ROSBUILD_ADD_EXECUTABLE( testCliente src/testCliente.cpp) 
TARGET_LINK_LIBRARIES( testCliente )
	

