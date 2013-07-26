#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_lib')
import rospy
from task_manager_lib.TaskClient import *
import argparse

server_node="/task_server"
default_period=0.1

parser = argparse.ArgumentParser(description='Print the list of tasks running on a given server node')
parser.add_argument('--server', '-s',default=server_node,
        nargs=1, help='server node name, e.g. /task_server')
parser.add_argument('--period', '-p',default=default_period,type=float, 
        nargs=1, help='default period for new tasks')
args = parser.parse_args()
# print args
default_period=args.period
server_node=args.server[0]

rospy.init_node('task_list')
server_node = rospy.get_param("~server",server_node)
default_period = rospy.get_param("~period",default_period)
tc = TaskClient(server_node,default_period)

tc.printTaskList()


