#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]



while True:
    try:
        # tc.Wait(duration=0.5)
        # Start the wait for roi task in the background
        w4roi = tc.Fail(foreground=False, iterations=0, error_type=4);

        # Prepare a condition so that the following gets executed only until the 
        # Region of Interest is found
        tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
        tc.SetPen(on=False)
        w4wait = tc.Wait(foreground=False, duration=5.0);
        tc.addCondition(ConditionIsCompleted("Wait detector",tc,w4wait))
        for p in wp:
            tc.Wait(duration=1.0)
        # Clear the conditions if we reach this point
        tc.clearConditions()
    except TaskConditionException as e:
        tc.get_logger().info("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        tc.Wait(duration=0.5)

    # Follow up with normal execution
    # tc.Wait(duration=2.)


tc.get_logger().info("Mission completed")


