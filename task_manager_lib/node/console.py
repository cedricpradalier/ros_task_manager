#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_lib')
import rospy
from task_manager_lib.TaskClient import *
from dynamic_reconfigure.encoding import *
import argparse

server_node=""
default_period=0.1
rospy.init_node('task_console')
server_node = rospy.get_param("~server",server_node)
default_period = rospy.get_param("~period",default_period)

parser = argparse.ArgumentParser(description='Print the list of tasks running on a given server node')
parser.add_argument('--server', '-s',default=server_node,required=(server_node==""),
        nargs=1, help='server node name, e.g. /task_server')
parser.add_argument('--period', '-p',default=default_period,type=float, 
        nargs=1, help='default period for new tasks')
args = parser.parse_args()
# print args
default_period=args.period
server_node=args.server[0]

tc = TaskClient(server_node,default_period)

def param_string(t):
    if t["type"]=="double" or t["type"]=="int":
        return "%s: %s in [%s,%s], default %s\n\t%s" % (t["name"],t["type"],\
                str(t["min"]),str(t["max"]),str(t["default"]),t["description"])
    else:
        return "%s: %s, default %s\n\t%s" % (t["name"],t["type"],\
                str(t["default"]),t["description"])

def index():
    global tc
    print "Known tasks summary:"
    for t in tc.tasklist.values():
        print "  %-16s: %s" % (t.name,t.help)
    print "Tasks name can be used as functions, e.g. Wait(duration=1.0)"
    print "Use help(Task) to get help on a specific task, e.g. help(Wait)"
    print "Type index() to display this summary"

for t in tc.tasklist.values():
    params=extract_params(decode_description(t.config))
    pnames=[p["name"] for p in params]
    pstring=[param_string(p) for p in params]

    f="def %s(**d):\n\t\"\"\"\n%s\n%s\n\"\"\"\n\tglobal tc;tc.%s(**d)\n" %\
            (t.name, t.help,"\n".join(pstring),t.name)
    exec(f)

del param_string
index()

