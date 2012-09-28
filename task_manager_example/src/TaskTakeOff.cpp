#include <assert.h>
#include "TaskTakeOff.h"

TaskTakeOff::TaskTakeOff(TaskEnvironment *env) : 
	/*             name      helpstring periodic timeout */
	TaskDefinition("TakeOff","Take Off",false,    30.0) 
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	interface = tce->simple;
}

TaskStatus TaskTakeOff::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskTakeOff::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Initialising...\n");
	return TASK_INITIALISED;
}

TaskStatus TaskTakeOff::iterate()
{
	int res;
	debug("Taking off to CTRLLED\n");
	taskStatus = TASK_RUNNING;
	res = sbSimpleReachNavState(interface, SB_NAV_CTRLLED, 30.0);
	if (res) {
		debug(" -= Task Failed %d =-\n",res);
		return TASK_FAILED;
	} else {
		debug(" -= Task Completed =-\n");
		return TASK_COMPLETED;
	}
}

TaskStatus TaskTakeOff::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskTakeOff);
