#ifndef TASK_SERVER_H
#define TASK_SERVER_H

#include <ros/ros.h>
#include "TaskScheduler.h"


class TaskServer
{
	protected:
		TaskScheduler & scheduler;
	public:
        ros::ServiceServer startTaskSrv;
        ros::ServiceServer stopTaskSrv;
        ros::ServiceServer getTaskListSrv;
        ros::ServiceServer getAllTaskStatusSrv;

		TaskServer(ros::NodeHandle & nh, TaskScheduler & ts);
		~TaskServer();

};



#endif // TASK_SERVER_H
