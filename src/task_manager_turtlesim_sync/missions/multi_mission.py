#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim_sync')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client1')
server1_node = rospy.get_param("~server1","/turtlesim_tasks1")
server2_node = rospy.get_param("~server2","/turtlesim_tasks2")
server3_node = rospy.get_param("~server3","/turtlesim_tasks3")
default_period = rospy.get_param("~period",0.2)
tc1 = TaskClient(server1_node,default_period)
tc2 = TaskClient(server2_node,default_period)
tc3 = TaskClient(server3_node,default_period)

print("Sequential orders")
tc1.GoTo(goal_x=1.0,goal_y=1.0)
tc2.GoTo(goal_x=3.0,goal_y=1.0)
tc3.GoTo(goal_x=5.0,goal_y=1.0)
top=1
while True:
    print("Joint orders")
    id1=tc1.GoTo(goal_x=1.0,goal_y=1.0+top*8,foreground=False)
    id2=tc2.GoTo(goal_x=3.0,goal_y=1.0+top*8,foreground=False)
    id3=tc3.GoTo(goal_x=5.0,goal_y=1.0+top*8,foreground=False)
    print("Waiting")
    tc1.waitTask(id1)
    tc2.waitTask(id2)
    tc3.waitTask(id3)
    top=1-top



rospy.loginfo("Multi Mission completed")


