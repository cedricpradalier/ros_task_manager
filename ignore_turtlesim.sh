#!/bin/bash

set -e
set -x
dir=`rospack find task_manager_lib`
cd "$dir"
cd ..
touch task_manager_turtlesim/CATKIN_IGNORE
touch task_manager_turtlesim_sync/CATKIN_IGNORE
touch task_manager_turtlesim_smach/CATKIN_IGNORE
touch turtlesim_logo/CATKIN_IGNORE
git add task_manager_turtlesim/CATKIN_IGNORE
git add task_manager_turtlesim_sync/CATKIN_IGNORE
git add task_manager_turtlesim_smach/CATKIN_IGNORE
git add turtlesim_logo/CATKIN_IGNORE

