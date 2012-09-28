#include <math.h>
#include <assert.h>
#include "TaskAltitude.h"

TaskAltitude::TaskAltitude(TaskEnvironment *env) : 
	TaskDefinition("Altitude", /* task name */
			/* Help string */
			"Go to desired altitude (args: altitude, precision)",
			/* periodic  timeout */
			true,        30.0) 
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	desired_altitude = -1;
	precision = 0.05;
	interface = tce->simple;
}

TaskStatus TaskAltitude::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskAltitude::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Initialising...\n");
	desired_altitude = parameters.doubleParam("altitude",desired_altitude);
	precision = parameters.doubleParam("precision",precision);
	if (desired_altitude < 0) {
		debug("Altitude must be positive and specified in the parameter list\n");
		return TASK_INITIALISATION_FAILED;
	}
	int res = sbConfigureControl(&interface->control,
			SB_CTRL_POS,SB_CTRL_POS,SB_CTRL_POS,SB_CTRL_REL);
	if (res) {
		debug("Failed to configure control: %d\n",res);
		return TASK_INITIALISATION_FAILED;
	}
	res = sbSimpleWaitState(interface,NULL,1.0);
	if (res) {
		debug("Failed to get first state: %d\n",res);
		return TASK_INITIALISATION_FAILED;
	}
	yawinit = interface->state.yaw;
	debug("Reaching altitude %.2f\n",desired_altitude);
	return TASK_INITIALISED;
}

TaskStatus TaskAltitude::iterate()
{
	int res;
	if (*(interface->endP)) {
		debug("Interrupted");
		return TASK_INTERRUPTED;
	}
	res = sbSimpleWaitState(interface,NULL,1.0);
	if (res) {
		debug("Failed to receive state: %d\n",res);
		return TASK_FAILED;
	}
	if (fabs(interface->state.zrange - desired_altitude) < precision) {
		debug("Altitude Reached: %.2f\n",interface->state.zrange);
		return TASK_COMPLETED;
	}
	res = sbSimpleControl(interface,0,0,yawinit,desired_altitude);
	if (res) {
		debug("Failed to set control: %d\n",res);
		return TASK_FAILED;
	}
	return TASK_RUNNING;
}

TaskStatus TaskAltitude::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskAltitude);
