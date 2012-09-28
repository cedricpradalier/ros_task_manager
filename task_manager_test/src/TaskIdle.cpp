#include "TaskIdle.h"

TaskStatus TaskIdle::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskIdle::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("initialising...\n");
	return TASK_INITIALISED;
}

TaskStatus TaskIdle::iterate()
{
	debug("Idling...\n");
	return TASK_RUNNING;
}

TaskStatus TaskIdle::terminate()
{
	debug("terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskIdle);
