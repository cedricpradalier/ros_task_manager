#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from math import *
from task_manager_lib.TaskSmach import *

rospy.init_node('task_client')

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]

# Create a SMACH state machine
mi = MissionStateMachine(period = 0.2)
sm = mi.Sequence()

# Add states to the container
with sm:
    init = sm.add("Wait", duration=1.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=1.0, goal_y=1.0)
    sm.add("Clear")
    for i,p in enumerate(wp):
        sm.add("Wait", duration=0.2)
        sm.add("SetPen", on=True, r=p[3], g=p[4], b=p[5])
        sm.add("GoTo", goal_x=p[0], goal_y=p[1], task_timeout=22.0-5*i,
                transitions={'TASK_COMPLETED':"Epsilon%02d"%i,'TASK_TIMEOUT':'Recovery%02d'%i})
        sm.add("ReachAngle", label="Recovery%02d"%i, target=pi/2)
        mi.epsilon_task("Epsilon%02d"%i)

    sm.add("Wait", duration=2.0)
    sm.add("SetPen", on=False)
    sm.add("GoTo", goal_x=5.0, goal_y=5.0)
    sm.add("ReachAngle", target=-pi/2, transitions={'TASK_COMPLETED':init})


mi.run(sm)

rospy.loginfo("Mission completed")