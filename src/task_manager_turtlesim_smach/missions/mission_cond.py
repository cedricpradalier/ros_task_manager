#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from math import *
from task_manager_lib.TaskSmach import *

rospy.init_node('task_client')
mi = MissionStateMachine()

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]

# Create a SMACH state machine
sm = mi.Sequence()

# Add states to the container
with sm:
    init = sm.add("Wait",duration=1.0)
    sm.add("SetPen",on=False)
    sm.add("GoTo",goal_x=1.0,goal_y=1.0)
    sm.add("Clear")

    sm_con = mi.Concurrence()
    with sm_con:
        sm_con.add("WaitForROI",roi_x=9.,roi_y=6.,roi_radius=1.0)

        sm_sub = mi.Sequence()
        with sm_sub:
            for i,p in enumerate(wp):
                sm_sub.add("Wait",duration=0.2)
                sm_sub.add("SetPen",on=True,r=p[3],g=p[4],b=p[5])
                sm_sub.add("GoTo",goal_x=p[0],goal_y=p[1])
        sm_con.add('normal_seq',sm_sub)

    sm.add('Concurrence',sm_con)
    sm.add("Wait",duration=2.0)
    sm.add("SetPen",on=False)
    sm.add("GoTo",goal_x=5.0,goal_y=5.0)
    sm.add("ReachAngle",target=-pi/2,transitions={'TASK_COMPLETED':init})

mi.run(sm)

rospy.loginfo("Mission completed")






# TODO: Remove when new exemple accepted

# # Create a SMACH state machine
# sm = mi.createSequence()

# # Add states to the container
# with sm:
#     init = mi.seq_task("Wait",duration=1.0)
#     mi.seq_task("SetPen",on=False)
#     mi.seq_task("GoTo",goal_x=1.0,goal_y=1.0)
#     mi.seq_task("Clear")

#     sm_con = mi.createConcurrence('normal_seq')
#     with sm_con:
#         mi.concurrent_task("WaitForROI",foreground=False,roi_x=9.,roi_y=6.,roi_radius=1.0)
#         sm_sub = mi.createSequence()
#         with sm_sub:
#             for i,p in enumerate(wp):
#                 mi.seq_task("Wait",duration=0.2)
#                 mi.seq_task("SetPen",on=True,r=p[3],g=p[4],b=p[5])
#                 mi.seq_task("GoTo",goal_x=p[0],goal_y=p[1])
#         smach.Concurrence.add('normal_seq',sm_sub)

#     smach.Sequence.add('Concurrence',sm_con)
#     mi.seq_task("Wait",duration=2.0)
#     mi.seq_task("SetPen",on=False)
#     mi.seq_task("GoTo",goal_x=5.0,goal_y=5.0)
#     mi.seq_task("ReachAngle",target=-pi/2,transitions={'TASK_COMPLETED':init})

# mi.run(sm)

# rospy.loginfo("Mission completed")
