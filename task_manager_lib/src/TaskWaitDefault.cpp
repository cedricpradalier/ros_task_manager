#include "task_manager_lib/TaskWaitDefault.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;

TaskIndicator TaskWaitDefault::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskWaitDefault::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    cfg = parameters.toConfig<TaskWaitConfig>();
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

TaskIndicator TaskWaitDefault::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}


// DYNAMIC_TASK(TaskWaitDefault);
