#!/usr/bin/python

import time
import TaskClient

tc = TaskClient.TaskClient('localhost')
tc.updateTaskList()
tc.printTaskList()

tc.updateTaskStatus()
tc.printTaskStatus()

print "-----------------"

param={'task_name':'Test','task_duration':5}
tc.startTaskAndWait(param)
tc.printTaskStatus()

tc.Long(task_duration=3,main_task=False)
for i in range(1,5):
	print "I'm doing something else"
	tc.updateTaskStatus()
	tc.printTaskStatus()
	time.sleep(1)


tc.close()


