#include <assert.h>
#include "TaskServo.h"

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)

TaskServo::TaskServo(TaskEnvironment *env) : 
	TaskDefinition("Servo", /* task name */
			/* Help string */
			"Go to desired distance of an obstacle (args: distance, sensor, precision, in meter)",
			/* periodic  timeout */
			true,        30.0),
	pid("dctrl",0.5, 0.5, 0)
{
	TaskCoaxEnvironment *tce = dynamic_cast<TaskCoaxEnvironment*>(env);
	assert(tce != NULL);
	sensor = -1;
	desired_distance = -1;
	precision = 0.03;
	interface = tce->simple;
}

TaskStatus TaskServo::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TASK_CONFIGURED;
}

TaskStatus TaskServo::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Initialising...\n");
	if (!parameters.hasParam("sensor")) {
		debug("Sensor must be specified in the parameter list");
		return TASK_INITIALISATION_FAILED;
	}
	if (!parameters.hasParam("distance")) {
		debug("Sensor must be specified in the parameter list");
		return TASK_INITIALISATION_FAILED;
	}
	desired_distance = parameters.doubleParam("distance",desired_distance);
	sensor = parameters.doubleParam("sensor",sensor);
	precision = parameters.doubleParam("precision",precision);
	if ((sensor < 1) || (sensor > 4)) {
		debug("Sensor must between 1 and 4");
		return TASK_INITIALISATION_FAILED;
	}
	if (desired_distance < 0) {
		debug("Servoing distance must be positive");
		return TASK_INITIALISATION_FAILED;
	}


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
	yawinit = interface->state.yaw;
	pid.reset();
	pid.setImax(0.1);
	pid.setOmax(0.5);
	debug("Reaching Distance %.2f\n",desired_distance);
	return TASK_INITIALISED;
}

TaskStatus TaskServo::iterate()
{
	int res;

	if (*(interface->endP)) {
		debug("Interrupted");
		return TASK_INTERRUPTED;
	}
	res = sbSimpleWaitState(interface,NULL,1.0);
	if (res) {
		debug("Failed to receive state: %d",res);
		return TASK_FAILED;
	}
	if (fabs(interface->state.hranges[sensor] - desired_distance) < precision) {
		debug("Servo Reached: %.2f",interface->state.hranges[sensor]);
		return TASK_COMPLETED;
	}

	double pitch=0,roll=0;
	switch (sensor) {
		case 0:
			pitch = +pid(interface->state.hranges[sensor]-desired_distance);
			break;
		case 1:
			pitch = -pid(interface->state.hranges[sensor]-desired_distance);
			break;
		case 2:
			roll = +pid(interface->state.hranges[sensor]-desired_distance);
			break;
		case 3:
			roll = -pid(interface->state.hranges[sensor]-desired_distance);
			break;
		default:
			pitch = roll = 0;
			break;
	}
	// debug("Control: %.2f %.2f\n",roll,pitch);
	res = sbSimpleControl(interface,D2R(roll),D2R(pitch),yawinit,altinit);
	if (res) {
		debug("Failed to set control: %d",res);
		return TASK_FAILED;
	}
	return TASK_RUNNING;
}

TaskStatus TaskServo::terminate()
{
	debug("Terminating...\n");
	return TASK_TERMINATED;
}

DYNAMIC_TASK(TaskServo);
