#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim_sync')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client2')
server_node = rospy.get_param("~server","/turtlesim_tasks2")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)

tc.SetStatusSync(status=0);
tc.WaitForStatusSync(partner="partner1",status=1);
tc.GoTo(goal_x=9.0,goal_y=1.0)
tc.SetStatusSync(status=1);
tc.WaitForStatusSync(partner="partner3",status=1);
tc.WaitForStatusSync(partner="partner1",status=0);
tc.GoTo(goal_x=9.0,goal_y=9.0)
tc.SetStatusSync(status=0);



rospy.loginfo("Mission2 completed")


