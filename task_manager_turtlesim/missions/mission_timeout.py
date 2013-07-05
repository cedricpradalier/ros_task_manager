#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_turtlesim')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/turtlesim_tasks")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]



while True:
    tc.Wait(duration=1.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=1.0,goal_y=1.0)
    tc.Clear()

    try:
        for p in wp:
            tc.Wait(duration=0.2)
            tc.ReachAngle(target=p[2])
            tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
            tc.GoTo(goal_x=p[0],goal_y=p[1],task_timeout=2.0)
    except TaskException, e:
        if e.status == TaskStatus.TASK_TIMEOUT:
            rospy.loginfo("Path following interrupted by timeout, as expected: %s" % str(e))
            tc.ReachAngle(target=pi/2)
        else:
            raise

    # Follow up with normal execution
    tc.Wait(duration=2.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=5.0,goal_y=5.0)
    tc.ReachAngle(target=pi/2)



rospy.loginfo("Mission completed")


