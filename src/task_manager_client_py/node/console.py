#!/usr/bin/python
# ROS specific imports
import sys
import rclpy
from task_manager_client_py.TaskClient import *
import argparse
import signal

    
rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

def signal_handler(signal, frame):
    global tc
    print("Killing all tasks by stopping the keep-alive pulse")
    tc.stopAllTasks()

signal.signal(signal.SIGINT,signal_handler)

def status():
    global tc
    tc.printTaskStatus()


def index():
    global tc
    print("Known tasks summary:")
    for t in tc.tasklist.values():
        print("  %-16s: %s" % (t.name,t.help))
    print( "Tasks name can be used as functions, e.g. Wait(duration=1.0)")
    print( "Use help(Task) to get help on a specific task, e.g. help(Wait)")
    print( "Use Ctrl-C to stop the keep-alive thread and kill all tasks")
    print( "Type status() to display the status of currently running tasks")
    print( "Type index() to display this summary")

def param_string(C):
    return "  %s (T %d, ro %s) : %s" % (C.name,C.type,str(C.read_only),C.description)

for t in tc.tasklist.values():
    pstring=[param_string(p) for p in t.config]

    f="def %s(**d):\n\t\"\"\"\n%s\n%s\n\"\"\"\n\tglobal tc;tc.%s(**d)\n" %\
            (t.name, t.help,"\n".join(pstring),t.name)
    exec(f)

del param_string
index()



