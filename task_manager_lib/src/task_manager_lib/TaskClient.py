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

import time
import socket
import sys

class TaskException(Exception):
    def __init__(self, value,id=None,status=None):
        self.value = value
	self.id = id
	self.status = status
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

    taskStatusList = [	'TASK_NEWBORN', 'TASK_CONFIGURED', 'TASK_INITIALISED',\
            'TASK_RUNNING', 'TASK_COMPLETED', 'TASK_TERMINATED', 'TASK_INTERRUPTED',\
            'TASK_FAILED', 'TASK_TIMEOUT', 'TASK_CONFIGURATION_FAILED', \
            'TASK_INITIALISATION_FAILED']
    taskStatusStrings = dict(enumerate(taskStatusList))
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

    class TaskDefinition:
        name = ""
        help = ""
        client = None
        def __init__(self,name,help,client):
            self.name = name
            self.help = help
            self.client = client

        def __call__(self,**paramdict):
            paramdict['task_name'] = self.name
            foreground = True
            if ('main_task' in paramdict):
                foreground = bool(paramdict['main_task'])
            if (foreground):
                rospy.loginfo("Starting task %s in foreground" % self.name)
                res = self.client.startTaskAndWait(paramdict)
                return res
            else:
                rospy.loginfo("Starting task %s in background: %d" % (self.name,id))
                id = self.client.startTask(paramdict)
                return id

    class TaskStatus:
        id = 0
        name = ""
        status = 0
        foreground = True
        statusString = ""
        statusTime = 0.0

        def __str__(self):
            output = "%f %-12s " % (self.statusTime,self.name)
            if (self.foreground):
                output += "F "
            else:
                output += "B "
            output += TaskClient.taskStatusStrings[self.status]
            output += ":" + self.statusString
            return output

    def __init__(self,server_node,default_period):
        self.statusLock = threading.RLock()
        self.statusCond = threading.Condition(self.statusLock)
        self.server_node = server_node
        self.default_period = default_period
        rospy.loginfo("Creating link to services on node " + self.server_node)
        # Bad hard-coding. TODO: get that from the status message def
        self.taskStatusStrings[128] = "TASK_TERMINATED"
        self.taskStatusId["TASK_TERMINATED"] = 128
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
            rospy.logerr("Service initialisation failed: %s"%e)
            raise

        self.keepAlivePub = rospy.Publisher(self.server_node + "/keep_alive",std_msgs.msg.Header)
        self.statusSub = rospy.Subscriber(self.server_node + "/status",
                TaskStatus, self.status_callback)
        self.timer = rospy.timer.Timer(rospy.Duration(0.1),self.timerCallback)

        self.updateTaskList()
        self.updateTaskStatus()
        rospy.sleep(0.5)



    def __del__(self):
        self.idle()

    def __getattr__(self,name):
        return self.tasklist[name]


    def timerCallback(self,timerEvent):
        if self.keepAlive:
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            self.keepAlivePub.publish(header)

    def updateTaskList(self):
        try:
            resp = self.get_task_list()
            self.tasklist = {}
            for t in resp.tlist:
                self.tasklist[t.name] = self.TaskDefinition(t.name,t.description,self)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

    def printTaskList(self):
        for k,v in self.tasklist.iteritems():
            print "Task %s: %s" % (k,v.help)


    def startTask(self,paramdict,name="",foreground=True,period=-1):
        if period < 0:
            period = self.default_period
        try:
            if ('task_name' in paramdict):
                name=paramdict['task_name']
                del paramdict['task_name']
            if ('main_task' not in paramdict):
                paramdict['main_task'] = bool(foreground)
            else:
                paramdict['main_task'] = bool(paramdict['main_task'])
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

    def status_callback(self,t):
        with self.statusLock:
            ts = self.TaskStatus()
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
            return self.taskstatus[taskId].status >= self.taskStatusId['TASK_TERMINATED']

    def updateTaskStatus(self):
        try:
            with self.statusLock:
                resp = self.get_status()
                for t in resp.running_tasks + resp.zombie_tasks:
                    ts = self.TaskStatus()
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
            rospy.logerr( "Service call failed: %s"%e)
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
                t1 = rospy.Time.now().to_sec()
                if (self.anyConditionVerified()):
                    for id in ids:
                        self.stopTask(id)
                    trueConditions = self.getVerifiedConditions();
                    self.clearConditions()
                    rospy.loginfo("Task %s terminated on condition" % str(ids))
                    raise TaskConditionException("Task %d terminated on condition" % id,trueConditions)
                for id in ids:
                    if id not in self.taskstatus:
                        if (t1-t0) > 1.0: 
                            if (self.verbose):
                                rospy.logerr("Id %d not in taskstatus" % id)
                            raise TaskException("Task %d did not appear in task status" % id,id);
                    else:
                        # print "%d: %02X" % (id, self.taskstatus[id].status)
                        if not (self.taskstatus[id].status & statusTerminated):
                            continue
                        status = self.taskstatus[id].status & (~statusTerminated)
                        if (status == self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                rospy.loginfo("Task %d terminated (%d)" % (id,status))
                            completed[id] = True
                        elif (status > self.taskStatusId["TASK_COMPLETED"]):
                            if (self.verbose):
                                rospy.logwarn( "Task %d failed (%d)" %  (id,status))
                            raise TaskException("Task %d failed: %s" % (id,self.taskStatusList[status]), id, status);
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
                raise TaskException("Aborting due to ROS shutdown");
            return False

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










