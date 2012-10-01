#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from task_manager_lib.TaskClient import *

tc = TaskClient("/turtlesim_tasks",0.1)
tc.printTaskList()
# tc.printTaskStatus()

tc.Wait(duration=1.)
tc.SetPen(on=False)
tc.GoTo(goal_x=1.0,goal_y=1.0)

tc.Wait(duration=1.)
tc.SetPen(on=True,r=0,g=0,b=255)
tc.GoTo(goal_x=1.0,goal_y=9.0)

tc.Wait(duration=1.)
tc.SetPen(on=True,r=0,g=255,b=255)
tc.GoTo(goal_x=9.0,goal_y=9.0)

tc.Wait(duration=1.)
tc.SetPen(on=True,r=0,g=255,b=0)
tc.GoTo(goal_x=9.0,goal_y=1.0)

tc.Wait(duration=1.)
tc.SetPen(on=True,r=255,g=255,b=0)
tc.GoTo(goal_x=1.0,goal_y=1.0)

tc.Wait(duration=1.)
tc.SetPen(on=False)
tc.GoTo(goal_x=5.0,goal_y=5.0)

tc.Wait(duration=1.)




