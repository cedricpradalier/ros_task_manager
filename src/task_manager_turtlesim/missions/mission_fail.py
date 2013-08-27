#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/turtlesim_tasks")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)

tc.Wait(duration=1.)

for error in [6,7,8,9,10]:
    try:
        tc.Fail(iterations=10, error_type=error);
    except TaskException, e:
        rospy.loginfo("Task fail with error status %d while expecting %d: %s" %(e.status, error, str(e)))

rospy.loginfo("Mission completed")


