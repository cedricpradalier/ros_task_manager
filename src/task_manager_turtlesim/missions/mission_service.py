#!/usr/bin/env python

from std_srvs.srv import *
import rospy
from task_manager_lib.TaskClient import *
from math import *


class MissionService:
    def __init__(self):
        rospy.init_node('task_client')
        server_node = rospy.get_param("~server","/turtlesim_tasks")
        default_period = rospy.get_param("~period",0.2)
        self.tc = TaskClient(server_node,default_period)
        self.mission_in_progress=False
        self.mission_spec=None
        self.service = rospy.Service('start_mission', SetBool, self.handle_mission_req)

    def handle_mission_req(self,req):
        if self.mission_in_progress:
            return SetBoolResponse(False,"I'm busy")
        self.mission_spec=req.data
        return SetBoolResponse(True,"Duly noted")

    def run_mission_left(self):
        wp = [[1., 1., pi, 0, 0, 255],
            [1., 9., pi/2, 0, 255, 0],
            [5., 9., 0, 255, 0, 0],
            [5., 5., -pi/2, 255, 255, 0]]
        self.tc.SetPen(on=False)
        self.tc.GoTo(goal_x=5.0,goal_y=1.0)
        for p in wp:
            self.tc.ReachAngle(target=p[2])
            self.tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
            self.tc.GoTo(goal_x=p[0],goal_y=p[1])
        self.tc.ReachAngle(target=pi/2)
        
    def run_mission_right(self):
        wp = [[9., 1., 0, 0, 0, 255],
            [9., 9., pi/2, 0, 255, 0],
            [5., 9., -pi, 255, 0, 0],
            [5., 5., -pi/2, 255, 255, 0]]
        self.tc.SetPen(on=False)
        self.tc.GoTo(goal_x=5.0,goal_y=1.0)
        for p in wp:
            self.tc.ReachAngle(target=p[2])
            self.tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
            self.tc.GoTo(goal_x=p[0],goal_y=p[1])
        self.tc.ReachAngle(target=pi/2)
        


    def run(self):
        rate=rospy.Rate(10)
        rospy.loginfo("Waiting for mission request")
        while not rospy.is_shutdown():
            if self.mission_spec is not None:
                self.mission_in_progress = True
                if self.mission_spec:
                    rospy.loginfo("Executing mission \"Left\"")
                    self.run_mission_left()
                else:
                    rospy.loginfo("Executing mission \"Right\"")
                    self.run_mission_right()
                self.mission_in_progress = False
                self.mission_spec = None
                rospy.loginfo("Waiting for next mission request")
            rate.sleep()

if __name__ == "__main__":
    mi = MissionService()
    mi.run()
