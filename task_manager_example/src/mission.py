#!/usr/bin/python

import TaskClient

tc = TaskClient.TaskClient('localhost')
tc.updateTaskList()
tc.verbose = True

try:

	tc.TakeOff()
	tc.Altitude(altitude=0.4)
	tc.Yaw(yaw=0)
	tc.Servo(distance=0.5,sensor=1)
	tc.Servo(distance=0.9,sensor=1)
	tc.Yaw(yaw=90)
	tc.Land()

except KeyboardInterrupt:
	print "Interrupted"

tc.idle()



