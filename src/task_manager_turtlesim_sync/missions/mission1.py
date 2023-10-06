#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks1', 0.2)

tc.Spawn(x=4,y=4,theta=0,name="turtle2")
tc.Spawn(x=7,y=7,theta=0,name="turtle3")
tc.SetStatusSync(status=0);
tc.GoTo(goal_x=1.0,goal_y=1.0)
tc.SetStatusSync(status=1);
tc.WaitForStatusSync(partner="partner2",status=1);
tc.WaitForStatusSync(partner="partner3",status=1);
tc.GoTo(goal_x=5.0,goal_y=5.0)
tc.SetStatusSync(status=0);



tc.get_logger().info("Mission1 completed")


