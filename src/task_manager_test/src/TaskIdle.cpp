#include "task_manager_test/TaskIdle.h"
using namespace task_manager_msgs;
using namespace task_manager_test;

TaskIndicator TaskIdle::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("configuring...\n");
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskIdle::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("initialising...\n");
	return Parent::initialise(parameters);
}

TaskIndicator TaskIdle::iterate()
{
    printf("+");fflush(stdout);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskIdle::terminate()
{
	debug("\nterminating...\n");
	return TaskStatus::TASK_TERMINATED;
}


DYNAMIC_TASK(TaskIdle);
