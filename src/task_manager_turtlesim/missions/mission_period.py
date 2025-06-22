#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

periods=[0.020, 0.200, 0.500, 1.000, 2.000]


for p in periods:
    tc.SetPen(on=False)
    tc.GoTo(goal_x=8.0,goal_y=2.0,task_period=p)
    tc.ReachAngle(target=pi/2)
    tc.SetPen(on=True)
    tc.Clear()
    tc.Ramp(linear_min=0.5,linear_max=1.5, angular_min=0.0, angular_max=1.0, duration=30.0, task_period=p)



tc.get_logger().info("Mission completed")


