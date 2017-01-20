#!/bin/bash

source ../../devel/setup.bash

xterm -e roslaunch turtlesim_logo launch_all.launch &
rosrun turtlesim_logo console
kill %1

