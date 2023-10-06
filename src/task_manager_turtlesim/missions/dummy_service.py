#!/usr/bin/env python3

from std_srvs.srv import *
import rospy

def handle_set_bool(req):
    print "Setting a bool... it is hard"
    rospy.sleep(5.0)
    print "Success!"
    return SetBoolResponse(True,"Thanks")

def set_bool_server():
    rospy.init_node('dummy_set_bool')
    s = rospy.Service('set_bool', SetBool, handle_set_bool)
    print "Ready to set bool."
    rospy.spin()

if __name__ == "__main__":
    set_bool_server()
