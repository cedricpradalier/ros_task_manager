
#include <sys/time.h>
#include <ros/ros.h>
#include "task_manager_lib/TaskHistoric.h"
#include <sys/time.h>
#include <ros/ros.h>
using namespace task_manager_lib;

TaskHistoric::TaskHistoric(unsigned int id, const std::string & name, TaskParameters config, const ros::Time & tnow, const unsigned int & statusnb)
{
	thid=id;
	thname=name;
	startTime=tnow;
	params=config;
	status=statusnb;
	endTime=tnow;
	
}
				
void TaskHistoric::updateTaskHistoric(ros::Time tnow, const unsigned int & statusnb)
{

	setendTime(tnow);
	setstatus(statusnb);
}

unsigned int TaskHistoric::getid()
{
	return thid;
}

std::string TaskHistoric::getname()
{
	return thname;
}

const ros::Time TaskHistoric::getstartTime()
{
	return startTime;
}

const ros::Time TaskHistoric::getendTime()
{
	return endTime;
}

TaskParameters TaskHistoric::getparams()
{
	return params;
}

unsigned int TaskHistoric::getstatus()
{
	return status;
}


void TaskHistoric::setid(unsigned int & current_id)
{
	thid=current_id;
}

void TaskHistoric::setname(std::string& current_name)
{
	thname=current_name;
}

void TaskHistoric::setstartTime(ros::Time time)
{
	startTime=time;
}

void TaskHistoric::setendTime(ros::Time time)
{
	endTime=time;
}

void TaskHistoric::setparams(TaskParameters& current_params)
{
	params=current_params;
}

void TaskHistoric::setstatus(const unsigned int & statusnb)
{
	status=statusnb;
}


