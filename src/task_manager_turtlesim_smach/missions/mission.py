#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim_smach')
import rospy
from math import *
from task_manager_lib.TaskSmach import *

rospy.init_node('task_client')

wp = [ [1., 9., 0, 0, 255],
    [9., 9., 0, 255, 255],
    [9., 1., 0, 255, 0],
    [1., 1., 255, 255, 0]]

# Create a SMACH state machine
mi = MissionStateMachine(period = 0.2)
sm = mi.Sequence()


# Add states to the container
with sm:
    sm.add("Wait", duration=1.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=1.0, goal_y=1.0)
    for p in wp:
        sm.add("Wait", duration=0.2)
        sm.add("SetPen", on=True, r=p[2], g=p[3], b=p[4])
        sm.add("GoTo", goal_x=p[0], goal_y=p[1])

    sm.add("Wait", duration=2.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=5.0, goal_y=5.0)
    sm.add("ReachAngle", target=pi/2)

mi.run(sm)

rospy.loginfo("Mission completed")
