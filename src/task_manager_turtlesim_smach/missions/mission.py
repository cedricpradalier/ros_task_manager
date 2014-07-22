#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim_smach')
import rospy
from math import *
from task_manager_lib.TaskSmach import *

rospy.init_node('task_client')
mi = MissionStateMachine()


wp = [ [1., 9., 0, 0, 255],
    [9., 9., 0, 255, 255],
    [9., 1., 0, 255, 0],
    [1., 1., 255, 255, 0]]

# Create a SMACH state machine
sm = mi.createSequence()

# Add states to the container
with sm:
    mi.seq_task("Wait",duration=1.0)
    mi.seq_task("SetPen",on=False)
    mi.seq_task("GoTo",goal_x=1.0,goal_y=1.0)
    for p in wp:
        mi.seq_task("Wait",duration=0.2)
        mi.seq_task("SetPen",on=True,r=p[2],g=p[3],b=p[4])
        mi.seq_task("GoTo",goal_x=p[0],goal_y=p[1])

    mi.seq_task("Wait",duration=2.0)
    mi.seq_task("SetPen",on=False)
    mi.seq_task("GoTo",goal_x=5.0,goal_y=5.0)
    mi.seq_task("ReachAngle",target=pi/2)

mi.run(sm)

rospy.loginfo("Mission completed")


