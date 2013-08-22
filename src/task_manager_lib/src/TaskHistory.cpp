
#include <sys/time.h>
#include <ros/ros.h>
#include "task_manager_lib/TaskHistory.h"
#include <sys/time.h>
#include <ros/ros.h>
using namespace task_manager_lib;

TaskHistory::TaskHistory(unsigned int id, const std::string & name, TaskParameters config, const ros::Time & tnow, const unsigned int & statusnb)
{
	thid=id;
	thname=name;
	startTime=tnow;
	params=config;
	status=statusnb;
	endTime=tnow;
	
}
				
void TaskHistory::updateTaskHistory(ros::Time tnow, const unsigned int & statusnb)
{

	setendTime(tnow);
	setstatus(statusnb);
}

unsigned int TaskHistory::getid()
{
	return thid;
}

std::string TaskHistory::getname()
{
	return thname;
}

const ros::Time TaskHistory::getstartTime()
{
	return startTime;
}

const ros::Time TaskHistory::getendTime()
{
	return endTime;
}

TaskParameters TaskHistory::getparams()
{
	return params;
}

unsigned int TaskHistory::getstatus()
{
	return status;
}


void TaskHistory::setid(unsigned int & current_id)
{
	thid=current_id;
}

void TaskHistory::setname(std::string& current_name)
{
	thname=current_name;
}

void TaskHistory::setstartTime(ros::Time time)
{
	startTime=time;
}

void TaskHistory::setendTime(ros::Time time)
{
	endTime=time;
}

void TaskHistory::setparams(TaskParameters& current_params)
{
	params=current_params;
}

void TaskHistory::setstatus(const unsigned int & statusnb)
{
	status=statusnb;
}


