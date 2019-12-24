#!/usr/bin/python
# ROS specific imports
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/turtlesim_tasks")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)


wp = [ (1.,1.), (1.,9.), (9.,9.), (9.,1.), (1.,1.) ] 

Action=tc.FollowPath.getActionDict("task_manager_turtlesim")

while True:
    tc.Wait(duration=1.)
    tc.Clear()
    tc.SetPen(on=False)
    tc.FollowPath(param_list_action=Action["Clear"])
    for x,y in wp:
        tc.FollowPath(goal_x=x,goal_y=y, param_list_action=Action["Push"])
    tc.FollowPath(param_list_action=Action["Execute"])

    tc.Wait(duration=1.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=5.0,goal_y=5.0)
    tc.ReachAngle(target=pi/2)

rospy.loginfo("Mission completed")


