#include "task_manager_lib/TaskIdleDefault.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;

TaskIndicator TaskIdleDefault::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskIdleDefault::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskIdleDefault::iterate()
{
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskIdleDefault::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}


// DYNAMIC_TASK(TaskIdleDefault);
