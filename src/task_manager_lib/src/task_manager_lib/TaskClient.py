# ROS specific imports
import roslib; roslib.load_manifest('task_manager_lib')
import threading
import rospy
import rospy.core
import rospy.timer
import std_msgs.msg
from task_manager_msgs.msg import *
from task_manager_lib.srv import *
from dynamic_reconfigure.encoding import *
import argparse

import time
import socket
import sys

class TaskException(Exception):
    def __init__(self, value,id=None,status=None,statusString=""):
        self.value = value
	self.id = id
	self.status = status
	self.statusString = statusString
    def __str__(self):
        return repr(self.value)

class TaskConditionException(Exception):
    def __init__(self, value, conds):
        self.value = value
        self.conditions = conds
    def __str__(self):
        return repr(self.value)

class Condition:
    def __init__(self,name):
        self.name = name
        test = self.isVerified()

    def __str__(self):
        return self.name

class NegatedCondition(Condition):
    def __init__(self,cond):
        self.name = "not " + cond.name
        self.cond = cond
    def isVerified(self):
        return not self.cond.isVerified()

class ConditionIsCompleted(Condition):
    def __init__(self, name, tc, taskId):
        self.name = name
        self.tc = tc
        self.taskId = taskId

    def isVerified(self):
        if not self.tc.isKnown(self.taskId):
            return False
        return self.tc.isCompleted(self.taskId) 

class ConditionIsRunning(Condition):
    def __init__(self, name, tc, taskId):
        self.name = name
        self.tc = tc
        self.taskId = taskId

    def isVerified(self):
        if not self.tc.isKnown(self.taskId):
            return False
        return not self.tc.isCompleted(self.taskId)


class TaskClient:
    sock = None
    verbose = 0
    messageid = 0
    keepAlive = False
    tasklist = {}
    taskstatus = {}
    conditions = []
    status_functions = []

    taskStatusStrings = dict([( TaskStatus.__dict__[k],k) for k in TaskStatus.__dict__.keys() if k[0:5]=="TASK_"])
    taskStatusId = dict([(v,k) for k,v in taskStatusStrings.iteritems()])

    def addCondition(self,cond):
        self.conditions.append(cond)

    def clearConditions(self):
        self.conditions = []

    def anyConditionVerified(self):
        return reduce(lambda x,y:x or y,[x.isVerified() for x in self.conditions],False)

    def allConditionsVerified(self):
        return reduce(lambda x,y:x and y,[x.isVerified() for x in self.conditions],False)

    def getVerifiedConditions(self):
        v = []
        for x in self.conditions:
            if x.isVerified():
                v.append(x)
        return v

    def registerStatusFunction(self,f):
        self.status_functions.append(f)

    class TaskDefinition:
        name = ""
        help = ""
        client = None
        def __init__(self,name,help,cfg,client):
            self.name = name
            self.help = help
            self.config = cfg
            params = extract_params(decode_description(self.config))
            self.params = dict([(p["name"],p) for p in params])
            for p in self.params.values():
                if p["type"]=="int":
                    p["conv"]=int
                if p["type"]=="double":
                    p["conv"]=float
                if p["type"]=="str":
                    p["conv"]=str
                if p["type"]=="bool":
                    p["conv"]=bool
            self.client = client

        def prepareParams(self,paramdict):
            for p in paramdict:
                if p not in self.params:
                    raise NameError("%s: Parameter '%s' is not declared for task '%s'" % (self.client.server_node,p,self.name))
                try:
                    paramdict[p] = self.params[p]["conv"](paramdict[p])
                except ValueError:
                    raise ValueError("%s: Could not convert argument '%s' from '%s' to '%s'"
                            % (self.client.server_node,p, str(paramdict[p]), self.params[p]["type"]))
                        
            paramdict['task_name'] = self.name
            return paramdict

        def start(self,**paramdict):
            paramdict = self.prepareParams(paramdict)
            id = self.client.startTask(paramdict)
            return id

        def __call__(self,**paramdict):
            paramdict = self.prepareParams(paramdict)
            foreground = True
            if ('foreground' in paramdict):
                foreground = bool(paramdict['foreground'])
            if (foreground):
                rospy.loginfo("%s: Starting task %s in foreground" % (self.client.server_node,self.name))
                res = self.client.startTaskAndWait(paramdict)
                return res
            else:
                id = self.client.startTask(paramdict)
                rospy.loginfo("%s: Starting task %s in background: %d" % (self.client.server_node,self.name,id))
                return id

    class TaskStatus:
        def __init__(self,client):
            self.client = client
            self.id = 0
            self.name = ""
            self.status = 0
            self.foreground = True
            self.statusString = ""
            self.statusTime = 0.0

        def __str__(self):
            output = "%f %-12s " % (self.statusTime,self.name)
            if (self.foreground):
                output += "F "
            else:
                output += "B "
            output += self.client.status_string(self.status)
            return output

    def __init__(self,server_node,default_period):
        self.statusLock = threading.RLock()
        self.statusCond = threading.Condition(self.statusLock)
        parser = argparse.ArgumentParser(description='Client to run and control tasks on a given server node')
        parser.add_argument('--server', '-s',default=server_node,required=(server_node==""),
                nargs=1, help='server node name, e.g. /task_server', type=str)
        parser.add_argument('--period', '-p',default=default_period,type=float, 
                nargs=1, help='default period for new tasks')
        parser.add_argument('--check', '-c',action='store_const', const=True, dest='check', default=False,  
                help='if set, only test task syntax, but do not run')
        args,unknown = parser.parse_known_args()
        # print args
        self.default_period=args.period
        if type(args.server) is list:
            self.server_node=args.server[0]
        else:
            self.server_node=args.server
        self.check_only=args.check

        rospy.loginfo("Creating link to services on node " + self.server_node)
        if self.check_only:
            rospy.loginfo("%s: Dry-run only: this might not work for complex mission"%self.server_node)
        try:
            rospy.wait_for_service(self.server_node + '/get_all_tasks')
            self.get_task_list = rospy.ServiceProxy(self.server_node + '/get_all_tasks', GetTaskList)
            rospy.wait_for_service(self.server_node + '/start_task')
            self.start_task = rospy.ServiceProxy(self.server_node + '/start_task', StartTask)
            rospy.wait_for_service(self.server_node + '/stop_task')
            self.stop_task = rospy.ServiceProxy(self.server_node + '/stop_task', StopTask)
            rospy.wait_for_service(self.server_node + '/get_all_status')
            self.get_status = rospy.ServiceProxy(self.server_node + '/get_all_status', GetAllTaskStatus)
        except rospy.ServiceException, e:
            rospy.logerr("%s: Service initialisation failed: %s"%(self.server_node,e))
            raise

        self.keepAlivePub = rospy.Publisher(self.server_node + "/keep_alive",std_msgs.msg.Header,queue_size=1)
        self.statusSub = rospy.Subscriber(self.server_node + "/status",
                TaskStatus, self.status_callback)
        self.timer = rospy.timer.Timer(rospy.Duration(0.1),self.timerCallback)

        self.updateTaskList()
        self.updateTaskStatus()
        rospy.sleep(0.5)
        self.idle()
        rospy.sleep(0.5)



    def __del__(self):
        self.idle()

    def __getattr__(self,name):
        if name=="__dir__":
            return self.tasklist.keys
        return self.tasklist[name]


    def timerCallback(self,timerEvent):
        if self.keepAlive and not rospy.is_shutdown():
            try:
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                self.keepAlivePub.publish(header)
            except rospy.ROSException, e:
                # Ignore, this sometimes happens on shutdown
                pass

    def updateTaskList(self):
        try:
            resp = self.get_task_list()
            self.tasklist = {}
            for t in resp.tlist:
                self.tasklist[t.name] = self.TaskDefinition(t.name,t.description,t.config,self)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

    def printTaskList(self):
        for k,v in self.tasklist.iteritems():
            print "Task %s: %s" % (k,v.help)


    def startTask(self,paramdict,name="",foreground=True,period=-1):
        if rospy.is_shutdown():
            raise TaskException("Aborting due to ROS shutdown")
        if self.check_only:
            rospy.sleep(0.5)
            return 0
        if period < 0:
            period = self.default_period
        try:
            if ('task_name' in paramdict):
                name=paramdict['task_name']
                del paramdict['task_name']
            if ('foreground' not in paramdict):
                paramdict['foreground'] = bool(foreground)
            else:
                paramdict['foreground'] = bool(paramdict['foreground'])
            if (period>0) and ('task_period' not in paramdict):
                paramdict['task_period'] = float(period)
            config = encode_config(paramdict)
            # print config
            # rospy.loginfo("Starting task %s" % name)
            resp = self.start_task(name,config)
            self.keepAlive = True
            return resp.id
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s"%e)
            raise

    def startTaskAndWait(self,paramdict,name="",foreground=True,period=-1.):
        tid = self.startTask(paramdict,name,foreground,period)
        if (self.verbose):
            rospy.logdebug( "Waiting task %d" % tid)
        if self.check_only:
            return True
        return self.waitTask(tid)

    def stopTask(self,id):
        try:
            resp = self.stop_task(id)
            return 0
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s"%e)
            raise

    def idle(self):
        try:
            resp = self.stop_task(-1)
            return 0
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s"%e)
            raise

    def status_string(self,v):
        statusTerminated = self.taskStatusId["TASK_TERMINATED"]
        if (v & statusTerminated):
            return self.taskStatusStrings[v & ~statusTerminated] + " & TASK_TERMINATED"
        else:
            return self.taskStatusStrings[v]

    def status_callback(self,t):
        with self.statusLock:
            ts = self.TaskStatus(self)
            ts.id = t.id
            ts.name = t.name
            status = t.status
            ts.status = status & 0xFFL
            ts.foreground = bool(status & 0x100)
            ts.statusString = t.status_string
            ts.statusTime = t.status_time.to_sec()
            self.taskstatus[ts.id] = ts

            t = rospy.Time.now().to_sec()
            to_be_deleted = []
            for k,v in self.taskstatus.iteritems():
                if (t - v.statusTime) > 10.0:
                    to_be_deleted.append(k)
            for k in to_be_deleted:
                del self.taskstatus[k]
            for f in self.status_functions:
                f(ts)
        try:
            self.statusCond.notify_all()
        except RuntimeError:
            # Nobody is waiting
            pass



    def isKnown(self,taskId):
        with self.statusLock:
            return taskId in self.taskstatus.keys()

    def isCompleted(self,taskId,requireKnown=True):
        with self.statusLock:
            if requireKnown and not self.isKnown(taskId):
                return False
            if not self.isKnown(taskId):
                return True
            return self.taskstatus[taskId].status & self.taskStatusId['TASK_TERMINATED']

    def updateTaskStatus(self):
        try:
            with self.statusLock:
                resp = self.get_status()
                for t in resp.running_tasks + resp.zombie_tasks:
                    ts = self.TaskStatus(self)
                    ts.id = t.id
                    ts.name = t.name
                    status = t.status
                    ts.status = status & 0xFFL
                    ts.foreground = bool(status & 0x100)
                    ts.statusString = t.status_string
                    ts.statusTime = t.status_time.to_sec()
                    self.taskstatus[ts.id] = ts
            try:
                self.statusCond.notify_all()
            except RuntimeError:
                # Nobody is waiting
                pass
        except rospy.ServiceException, e:
            rospy.logerr( "%s: Service call failed: %s"%(self.server_node,e))
            raise

    def waitTaskList(self,ids,wait_for_all, stop_others):
        statusTerminated = self.taskStatusId['TASK_TERMINATED']
        t0 = rospy.Time.now().to_sec()
        completed = dict([(id,False) for id in ids])
        red_fun = lambda x,y: x or y
        if wait_for_all:
            red_fun = lambda x,y: x and y

        with self.statusLock:
            while not rospy.core.is_shutdown():
                if self.verbose>1:
                    self.printTaskStatus()
                t1 = rospy.Time.now().to_sec()
                if (self.anyConditionVerified()):
                    for id in ids:
                        self.stopTask(id)
                    trueConditions = self.getVerifiedConditions();
                    self.clearConditions()
                    rospy.loginfo("%s: Task %s terminated on condition" % (self.server_node,str(ids)))
                    raise TaskConditionException("%s: Task %s terminated on condition" % (self.server_node,str(ids)),trueConditions)
                for id in ids:
                    if id not in self.taskstatus:
                        if (t1-t0) > 2.0: 
                            if (self.verbose):
                                rospy.logerr("%s: Id %d not in taskstatus" % (self.server_node,id))
                            raise TaskException("%s: Task %d did not appear in task status" % (self.server_node,id),id);
                    else:
                        if self.verbose>1:
                            print "%s: %d: %02X - %s\n%s" % (self.server_node,id, self.taskstatus[id].status,self.status_string(self.taskstatus[id].status),self.taskstatus[id].statusString)
                        if not (self.taskstatus[id].status & statusTerminated):
                            continue
                        status = self.taskstatus[id].status & (~statusTerminated)
                        if (status == self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                rospy.loginfo("%s: Task %d terminated (%d)" % (self.server_node,id,status))
                            completed[id] = True
                        elif (status > self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                rospy.logwarn( "%s: Task %d failed (%d - %s)" %  (self.server_node,id,status,self.status_string(status)))
                            raise TaskException("%s: Task %d:%s failed: %d:%s" % (self.server_node,id,self.taskstatus[id].name,status,self.status_string(status)), id, status,self.taskstatus[id].statusString);
                            # instead of raise?
                            # completed[id] = True
                if reduce(red_fun,completed.values()):
                    if stop_others:
                        for k,v in completed.iteritems():
                            if not v:
                                self.stopTask(k)
                    return True
                # This would be a good idea, except that python is not
                # receiving the messages while waiting for the condition
                self.statusCond.wait(0.020)
            if rospy.core.is_shutdown():
                raise TaskException("%s: Aborting due to ROS shutdown"%self.server_node);
            return False

    def stopAllTasks(self):
        self.keepAlive = False
        statusTerminated = self.taskStatusId['TASK_TERMINATED']
        for id in self.taskstatus:
            if self.taskstatus[id].status & (~statusTerminated):
                continue
            self.stopTask(id)

    def waitTask(self,id):
        return self.waitTaskList([id],True,False)

    def waitAnyTasks(self,ids,stop_others=True):
        return self.waitTaskList(ids,False,stop_others)

    def waitAllTasks(self,ids):
        return self.waitTaskList(ids,True,False)

    def printTaskStatus(self):
        with self.statusLock:
            for k,v in self.taskstatus.iteritems():
                print "Task %d: %s" % (k,str(v))










