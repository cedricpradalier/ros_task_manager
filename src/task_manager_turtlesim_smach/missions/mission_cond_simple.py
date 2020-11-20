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

# Add states to the container
with sm:
    init = sm.add("Wait", duration=1.0)
    sm.add("GoTo", goal_x=1.0, goal_y=1.0)

    sm_con = mi.Concurrence()
    with sm_con:
        sm_con.add("WaitForROI", roi_x=9., roi_y=6., roi_radius=1.0)

        sm_sub = mi.Sequence()
        with sm_sub:
            sm_sub.add("Wait", duration=0.2)
            sm_sub.add("GoTo", goal_x=1.0,goal_y=9.0)
        sm_con.add('normal_seq',sm_sub)

    sm.add('Concurrence', sm_con)
    sm.add("GoTo", goal_x=5.0, goal_y=5.0, transitions={'TASK_COMPLETED':init})

mi.run(sm)

rospy.loginfo("Mission completed")