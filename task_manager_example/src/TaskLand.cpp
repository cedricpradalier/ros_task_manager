#include <assert.h>
#include "TaskLand.h"

TaskLand::TaskLand(TaskEnvironment *env) : 
	/*             name      helpstring periodic timeout */
	TaskDefinition("Land","Land",false,    30.0) 
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	interface = tce->simple;
}

TaskStatus TaskLand::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskLand::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Initialising...\n");
	return TASK_INITIALISED;
}

TaskStatus TaskLand::iterate()
{
	int res;
	taskStatus = TASK_RUNNING;
	debug("Taking off\n");
	res = sbSimpleReachNavState(interface, SB_NAV_IDLE, 30);
	if (res) {
		debug(" -= Task Failed %d =-\n",res);
		return TASK_FAILED;
	} else {
		debug(" -= Task Completed =-\n");
		return TASK_COMPLETED;
	}
}

TaskStatus TaskLand::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskLand);
