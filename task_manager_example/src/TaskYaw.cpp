#include <math.h>
#include <assert.h>
#include "TaskYaw.h"

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)

TaskYaw::TaskYaw(TaskEnvironment *env) : 
	TaskDefinition("Yaw", /* task name */
			/* Help string */
			"Go to desired Yaw (args: yaw, precision, in degrees)",
			/* periodic  timeout */
			true,        30.0) 
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	desired_yaw = -1;
	precision = 2;
	interface = tce->simple;
}

TaskStatus TaskYaw::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskYaw::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Initialising...\n");
	if (!parameters.hasParam("yaw")) {
		debug("Yaw must be specified in the parameter list");
		return TASK_INITIALISATION_FAILED;
	}
	desired_yaw = parameters.doubleParam("yaw",desired_yaw);
	precision = parameters.doubleParam("precision",precision);

	int res = sbConfigureControl(&interface->control,
			SB_CTRL_POS,SB_CTRL_POS,SB_CTRL_POS,SB_CTRL_REL);
	if (res) {
		debug("Failed to configure control: %d",res);
		return TASK_INITIALISATION_FAILED;
	}
	res = sbSimpleWaitState(interface,NULL,1.0);
	if (res) {
		debug("Failed to get first state: %d",res);
		return TASK_INITIALISATION_FAILED;
	}
	altinit = interface->state.zrange;
	debug("Reaching Yaw %.2f\n",desired_yaw);
	return TASK_INITIALISED;
}

TaskStatus TaskYaw::iterate()
{
	int res;
	double dyaw = D2R(desired_yaw);
	double dpre = D2R(precision);

	if (*(interface->endP)) {
		debug("Interrupted");
		return TASK_INTERRUPTED;
	}
	res = sbSimpleWaitState(interface,NULL,1.0);
	if (res) {
		debug("Failed to receive state: %d",res);
		return TASK_FAILED;
	}
	if (fabs(remainder(interface->state.yaw - dyaw,2*M_PI)) < dpre) {
		debug("Yaw Reached: %.2f",R2D(interface->state.yaw));
		return TASK_COMPLETED;
	}
	res = sbSimpleControl(interface,0,0,dyaw,altinit);
	if (res) {
		debug("Failed to set control: %d",res);
		return TASK_FAILED;
	}
	return TASK_RUNNING;
}

TaskStatus TaskYaw::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskYaw);
