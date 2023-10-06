#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *
from std_srvs.srv import Trigger

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]

cancel_request = ConditionVariable("cancel request")


def handle_trigger(req,resp):
    global cancel_request
    cancel_request.set(True)
    print("Received cancel request")
    resp.success = True
    resp.message = "Done"
    return resp

s = tc.create_service(Trigger, 'cancel_request', handle_trigger)


while True:
    tc.Wait(duration=1.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=1.0,goal_y=1.0)
    tc.Clear()

    cancel_request.set(False)
    # Start the wait for roi task in the background
    w4roi = tc.WaitForROI(foreground=False,roi_x=3.,roi_y=9.,roi_radius=1.0)
    # Prepare a condition so that the following gets executed only until the 
    # Region of Interest is found
    tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
    tc.addCondition(cancel_request)
    try:
        for p in wp:
            tc.Wait(duration=0.2)
            tc.ReachAngle(target=p[2])
            tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
            tc.GoTo(goal_x=p[0],goal_y=p[1])
        # Clear the conditions if we reach this point
        tc.clearConditions()
    except TaskConditionException as e:
        tc.get_logger().info("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        tc.ReachAngle(target=-pi/2)

    # Follow up with normal execution
    tc.Wait(duration=2.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=5.0,goal_y=5.0)
    tc.ReachAngle(target=pi/2)



tc.get_logger().info("Mission completed")


