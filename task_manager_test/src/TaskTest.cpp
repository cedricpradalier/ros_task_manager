
#include "TaskTest.h"

TaskStatus TaskTest::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	this->setName(parameters.stringParam("task_rename",name));
	return TASK_CONFIGURED;
}

TaskStatus TaskTest::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	duration = parameters.longParam("task_duration",3);
	counter = 0;
	debug("Initialising (duration = %d)...\n",duration);
	return TASK_INITIALISED;
}

TaskStatus TaskTest::iterate()
{
	debug("Testing (%d/%d)...\n",counter,duration);
	counter += 1;
	if (counter >= duration) {
		debug(" -= Task Completed =-\n");
		return TASK_COMPLETED;
	}
	return TASK_RUNNING;
}

TaskStatus TaskTest::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskTest);
