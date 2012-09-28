#include <assert.h>
#include "TaskIdle.h"


TaskIdle::TaskIdle(TaskEnvironment *env) : 
	/*             name      helpstring periodic timeout */
	TaskDefinition("Idle","Do nothing",true,-1)
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	interface = tce->simple;
}

TaskStatus TaskIdle::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TASK_CONFIGURED;
}

TaskStatus TaskIdle::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	int res = sbConfigureControl(&interface->control,
			SB_CTRL_NONE,SB_CTRL_NONE,SB_CTRL_NONE,SB_CTRL_NONE);
	if (res) {
		debug("Failed to configure control: %d",res);
		return TASK_INITIALISATION_FAILED;
	}
	return TASK_INITIALISED;
}

TaskStatus TaskIdle::iterate()
{
	return TASK_RUNNING;
}

TaskStatus TaskIdle::terminate()
{
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskIdle);
