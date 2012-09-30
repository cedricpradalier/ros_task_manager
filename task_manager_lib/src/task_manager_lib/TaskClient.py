# ROS specific imports
import roslib; roslib.load_manifest('task_manager_lib')
import rospy
from task_manager_msgs.msg import *
from task_manager_lib.srv import *
from dynamic_reconfigure.encoding import *

import time
import socket
import sys

class TaskClient:
	sock = None
	verbose = 0
	messageid = 0
	tasklist = {}
	taskstatus = {}

	taskStatusList = [	'TASK_NEWBORN', 'TASK_CONFIGURED', 'TASK_INITIALISED',\
	'TASK_RUNNING', 'TASK_COMPLETED', 'TASK_TERMINATED', 'TASK_INTERRUPTED',\
	'TASK_FAILED', 'TASK_TIMEOUT', 'TASK_CONFIGURATION_FAILED', \
	'TASK_INITIALISATION_FAILED']
	taskStatusStrings = dict(enumerate(taskStatusList))
	taskStatusId = dict([(v,k) for k,v in taskStatusStrings.iteritems()])

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
				print "Starting task in foreground"
				return self.client.startTaskAndWait(paramdict)
			else:
				print "Starting task in background"
				return self.client.startTask(paramdict)

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

	def __init__(self,server_node):
        self.server_node = server_node
        try:
            rospy.wait_for_service(self.server_node + '/get_task_list')
            self.get_task_list = rospy.ServiceProxy(self.server_node + '/get_task_list', GetTaskList)
            rospy.wait_for_service(self.server_node + '/start_task')
            self.start_task = rospy.ServiceProxy(self.server_node + '/start_task', StartTask)
            rospy.wait_for_service(self.server_node + '/stop_task')
            self.stop_task = rospy.ServiceProxy(self.server_node + '/stop_task', StopTask)
            rospy.wait_for_service(self.server_node + '/start_task')
            self.get_task_status = rospy.ServiceProxy(self.server_node + '/get_task_status', GetAllTaskStatus)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        self.statusSub = rospy.Subscriber(self.server_node + "/status",
                TaskStatus, self.status_callback)

        self.updateTaskList()
        self.updateTaskStatus()



	def __del__(self):
		self.idle()

	def __getattr__(self,name):
		return self.tasklist[name]


	def updateTaskList(self):
        try:
            resp = self.get_task_list()
            self.tasklist = {}
            for t in resp.tlist:
                self.tasklist[t.name] = self.TaskDefinition(t.name,t.description,self)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

	def printTaskList(self):
		for k,v in self.tasklist.iteritems():
			print "Task %s: %s" % (k,v.help)
	

	def startTask(self,paramdict,name="",foreground=True,period=-1):
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
            if (self.verbose):
                print "Starting task %s" % paramdict['task_name']
            resp = self.start_task(name,config)
            return resp.id
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

	def startTaskAndWait(self,paramdict,name="",foreground=True,period=0):
		tid = self.startTask(paramdict,name,foreground,period)
		if (self.verbose):
			print "Waiting task %d" % tid
		return self.waitTask(tid)

	def idle(self):
        try:
            resp = self.stop_task(-1)
            return 0
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def status_callback(self,t):
        ts = self.TaskStatus()
        ts.id = t.id
        ts.name = t.name
        status = t.status
        ts.status = status & 0xFFL
        ts.foreground = bool(status & 0x100)
        ts.statusString = t.status_string
        ts.statusTime = t.status_time.to_sec()
        self.taskstatus[ts.id] = ts


	def updateTaskStatus(self):
        try:
            resp = self.get_status()
            for t in resp.running_tasks + zombie_tasks:
                ts = self.TaskStatus()
                ts.id = t.id
                ts.name = t.name
                status = t.status
                ts.status = status & 0xFFL
                ts.foreground = bool(status & 0x100)
                ts.statusString = t.status_string
                ts.statusTime = t.status_time.to_sec()
                self.taskstatus[ts.id] = ts
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


	def waitTask(self,id):
		statusTerminated = self.taskStatusId['TASK_TERMINATED']
		while True:
			time.sleep(0.020)
			if (id not in self.taskstatus):
				if (self.verbose):
					print "Id %d not in taskstatus" % id
				return False
			if (self.taskstatus[id].status == statusTerminated):
				if (self.verbose):
					print "Task %d terminated" % id
				return True
			if (self.taskstatus[id].status > statusTerminated):
				if (self.verbose):
					print "Task %d failed" % id
				return False
		return False

	def printTaskStatus(self):
		for k,v in self.taskstatus.iteritems():
			print "Task %d: %s" % (k,str(v))
	
	

	


			



