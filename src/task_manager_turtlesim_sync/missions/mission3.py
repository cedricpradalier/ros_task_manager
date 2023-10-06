#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks3', 0.2)

tc.SetStatusSync(status=0);
tc.WaitForStatusSync(partner="partner2",status=1);
tc.GoTo(goal_x=1.0,goal_y=9.0)
tc.SetStatusSync(status=1);
tc.WaitForStatusSync(partner="partner1",status=0);
tc.WaitForStatusSync(partner="partner2",status=0);
tc.GoTo(goal_x=1.0,goal_y=1.0)
tc.SetStatusSync(status=0);




tc.get_logger().info("Mission3 completed")

