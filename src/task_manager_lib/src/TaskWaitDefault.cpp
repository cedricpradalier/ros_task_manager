#include "task_manager_lib/TaskWaitDefault.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;

TaskIndicator TaskWaitDefault::initialise() 
{
    t0 = ros::Time::now();
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitDefault::iterate()
{
    ros::Duration d = ros::Time::now() - t0;
    if (d.toSec() > cfg.duration) {
        return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


// DYNAMIC_TASK(DefinitionWaitDefault);
