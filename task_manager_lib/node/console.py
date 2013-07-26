#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('task_manager_lib')
import rospy
from task_manager_lib.TaskClient import *
from dynamic_reconfigure.encoding import *
import argparse
import signal

    

server_node=""
default_period=0.1
rospy.init_node('task_console',disable_signals=False)
server_node = rospy.get_param("~server",server_node)
default_period = rospy.get_param("~period",default_period)

print "Node: " + str(server_node)
tc = TaskClient(server_node,default_period)
def signal_handler(signal, frame):
    global tc
    print "Killing all tasks by stopping the keep-alive pulse"
    tc.stopAllTasks()

signal.signal(signal.SIGINT,signal_handler)

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
    print "Use Ctrl-C to stop the keep-alive thread and kill all tasks"
    print "Type status() to display the status of currently running tasks"
    print "Type index() to display this summary"

def status():
    global tc
    tc.printTaskStatus()

for t in tc.tasklist.values():
    params=extract_params(decode_description(t.config))
    pnames=[p["name"] for p in params]
    pstring=[param_string(p) for p in params]

    f="def %s(**d):\n\t\"\"\"\n%s\n%s\n\"\"\"\n\tglobal tc;tc.%s(**d)\n" %\
            (t.name, t.help,"\n".join(pstring),t.name)
    exec(f)

del param_string
index()



