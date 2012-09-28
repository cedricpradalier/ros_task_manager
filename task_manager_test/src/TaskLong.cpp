
#include "TaskLong.h"

TaskStatus TaskLong::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	this->setName(parameters.stringParam("task_rename",name));
	return TASK_CONFIGURED;
}

TaskStatus TaskLong::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	duration = parameters.longParam("task_duration",3);
	counter = 0;
	debug("Initialising (duration = %d)...\n",duration);
	return TASK_INITIALISED;
}

TaskStatus TaskLong::iterate()
{
	char tmp[128];
	while (counter < duration) {
		taskStatus = TASK_RUNNING;
		sprintf(tmp,"Longing (%d/%d)...\n",counter,duration);
		setStatusString(tmp);
		debug(tmp);
		counter += 1;
		sleep(1);
	}
	debug(" -= Task completed =- \n");
	return TASK_COMPLETED;
}

TaskStatus TaskLong::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskLong);
