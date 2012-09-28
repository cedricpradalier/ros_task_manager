#include <assert.h>

#include "TaskServer.h"


TaskServer::TaskServer(ros::NodeHandle & nh, TaskScheduler & ts) :
	scheduler(ts)
{
    startTaskSrv = nh.advertiseService("start_task", &TaskServer::startTask,this);
    stopTaskSrv = nh.advertiseService("stop_task", &TaskServer::stopTask,this);
    getTaskListSrv = nh.advertiseService("get_all_tasks", &TaskServer::getTaskList,this);
    getAllTaskStatusSrv = nh.advertiseService("get_all_status", &TaskServer::getAllTaskStatus,this);
}

TaskServer::~TaskServer()
{
}

bool TaskServer::startTask(task_manager_lib::StartTask::Request  &req,
         task_manager_lib::StartTask::Response &res )
{
    TaskScheduler::TaskId id = scheduler.launchTask(req.name,req.config);
    res.id = id;
    return true;
}

bool TaskServer::stopTask(task_manager_lib::StopTask::Request  &req,
         task_manager_lib::StopTask::Response &res )
{
    TaskScheduler::TaskId id = scheduler.launchIdleTask();
    res.id = id;
    return true;
}

bool TaskServer::getTaskList(task_manager_lib::GetTaskList::Request  &req,
         task_manager_lib::GetTaskList::Response &res )
{
    scheduler.generateTaskList(res.tlist);
    return true;
}

bool TaskServer::getAllTaskStatus(task_manager_lib::GetTaskList::Request  &req,
         task_manager_lib::GetTaskList::Response &res )
{
    scheduler.generateTaskStatus(res.running_tasks,res.zombie_tasks);
    return true;
}


