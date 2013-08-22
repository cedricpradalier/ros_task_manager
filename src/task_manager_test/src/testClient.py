#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_test')
import rospy
from task_manager_lib.TaskClient import *



tc = TaskClient("/tasks",0.5)
tc.verbose = True
tc.printTaskList()
tc.printTaskStatus()

print "-----------------"

param={'task_name':'Test','task_duration':5.}
tc.startTaskAndWait(param)
tc.printTaskStatus()

tc.Test(task_duration=6.)

tc.Long(task_duration=4.,main_task=False)
for i in range(1,10):
	print "I'm doing something else"
	tc.printTaskStatus()
	rospy.sleep(1)



