#!/usr/bin/python

import TaskClient

tc = TaskClient.TaskClient('localhost')
tc.updateTaskList()
tc.printTaskList()


