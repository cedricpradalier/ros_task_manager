#!/usr/bin/python
# coding=utf8
# ROS specific imports
import roslib; roslib.load_manifest('turtlesim_logo')
import rospy
from task_manager_lib.TaskClient import *
from dynamic_reconfigure.encoding import *
import argparse
import signal
import math

    

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

def index_tasks():
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
# index()

def INIT():
    GoTo(goal_x=5,goal_y=5,relative=False)
    ReachAngle(target=0.,relative=False,k_alpha=5,threshold=1e-3)
    

def AV(d):
    GoTo(goal_x=d,goal_y=0,relative=True)

def TG(a):
    ReachAngle(target=a*math.pi/180.,relative=True,k_alpha=5,threshold=1e-3)

def TD(a):
    ReachAngle(target=-a*math.pi/180.,relative=True,k_alpha=5,threshold=1e-3)

def PAUSE(t):
    Wait(duration=t)
    
def EFFACE():
    Clear()

def LEVE():
    SetPen(on=False)

def COULEUR(red,green,blue):
    SetPen(on=True,r=red,g=green,b=blue,width=3)

def NOIR():
    SetPen(on=True,r=0,g=0,b=0,width=3)

def BLANC():
    SetPen(on=True,r=255,g=255,b=255,width=3)

def CYAN():
    SetPen(on=True,r=0,g=255,b=255,width=3)

def JAUNE():
    SetPen(on=True,r=255,g=255,b=0,width=3)

def ROSE():
    SetPen(on=True,r=255,g=0,b=255,width=3)

def ROUGE():
    SetPen(on=True,r=255,g=0,b=0,width=3)

def VERT():
    SetPen(on=True,r=0,g=255,b=0,width=3)

def BLEU():
    SetPen(on=True,r=0,g=0,b=255,width=3)

def AIDE():
    print("PAUSE(temps): Attend selon le temps demandé")
    print("AV(distance): Avance de la distance demandée")
    print("TG(angle)   : Tourne vers la gauche selon l'angle demandé (degré)")
    print("TD(angle)   : Tourne vers la droite selon l'angle demandé (degré)")
    print("EFFACE()    : Efface l'écran")
    print("LEVE()      : Leve le crayon")
    print("NOIR()      : Sélectionne et pose un crayon noir")
    print("BLANC()     : Sélectionne et pose un crayon blanc")
    print("CYAN()      : Sélectionne et pose un crayon cyan/turquoise")
    print("JAUNE()     : Sélectionne et pose un crayon jaune")
    print("ROSE()      : Sélectionne et pose un crayon rose")
    print("ROUGE()     : Sélectionne et pose un crayon rouge")
    print("VERT()      : Sélectionne et pose un crayon vert")
    print("BLEU()      : Sélectionne et pose un crayon bleu")
    print("COULEUR(rouge,vert,bleu): Pose le crayon et choisit une couleur")

# Same command in small letter
def init():
    INIT()

def av(d):
    AV(d)

def tg(a):
    TG(a)

def td(a):
    TD(a)

def pause(t):
    PAUSE(t)
    
def efface():
    EFFACE()

def leve():
    LEVE()

def couleur(red,green,blue):
    COULEUR(red,green,blue)

def noir():
    NOIR()

def blanc():
    BLANC()

def cyan():
    CYAN()

def jaune():
    JAUNE()

def rose():
    ROSE()

def rouge():
    ROUGE()

def vert():
    VERT()

def bleu():
    BLEU()

def aide():
    AIDE()

AIDE()



