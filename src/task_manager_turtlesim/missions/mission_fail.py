#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

tc.Wait(duration=1.)

for error in [6,7,8,9,10]:
    try:
        tc.Fail(iterations=10, error_type=error);
    except TaskException as e:
        tc.get_logger().warn("Task fail with error status %d while expecting %d: %s" %(e.status, error, str(e)))

tc.get_logger().info("Mission completed")


