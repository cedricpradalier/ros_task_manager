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
    except TaskConditionException, e:
        rospy.loginfo("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        tc.Wait(duration=0.5)

    # Follow up with normal execution
    # tc.Wait(duration=2.)


rospy.loginfo("Mission completed")


