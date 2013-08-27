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
    tc.Wait(duration=1.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=1.0,goal_y=1.0)
    tc.Clear()

    # Start the wait for roi task in the background
    w4roi = tc.WaitForROI(foreground=False,roi_x=1.,roi_y=6.,roi_radius=1.0)
    # Prepare a condition so that the following gets executed only until the 
    # Region of Interest is found
    tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
    try:
        for p in wp:
            tc.Wait(duration=0.2)
            tc.ReachAngle(target=p[2])
            tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
            tc.GoTo(goal_x=p[0],goal_y=p[1])
        # Clear the conditions if we reach this point
        tc.clearConditions()
    except TaskConditionException, e:
        rospy.loginfo("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        tc.ReachAngle(target=pi/2)

    # Follow up with normal execution
    tc.Wait(duration=2.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=5.0,goal_y=5.0)
    tc.ReachAngle(target=pi/2)



rospy.loginfo("Mission completed")


