#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from math import *
from task_manager_lib.TaskSmach import *

rospy.init_node('task_client')

# Create a SMACH state machine
mi = MissionStateMachine(period = 0.2)
sm = mi.Sequence()

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]


# Add states to the container
with sm:
    init = sm.add("Wait", duration=1.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=1.0, goal_y=1.0)
    sm.add("Clear")

    sm_con = mi.Concurrence()
    with sm_con:
        sm_con.add("WaitForROI", roi_x=9., roi_y=6., roi_radius=1.0)

        sm_sub = mi.Sequence()
        with sm_sub:
            for i,p in enumerate(wp):
                sm_sub.add("Wait", duration=0.2)
                sm_sub.add("SetPen", on=True, r=p[3], g=p[4], b=p[5])
                sm_sub.add("GoTo", goal_x=p[0], goal_y=p[1])
        sm_con.add('normal_seq', sm_sub)

    sm.add('Concurrence', sm_con)
    sm.add("Wait", duration=2.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=5.0, goal_y=5.0)
    sm.add("ReachAngle", target=-pi/2, transitions={'TASK_COMPLETED':init})

mi.run(sm)

rospy.loginfo("Mission completed")
