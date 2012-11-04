
#include "task_manager_test/TaskLong.h"
#include "task_manager_test/TaskLongConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_test;

TaskIndicator TaskLong::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskLong::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator parent = Parent::initialise(parameters);
    if (parent != TaskStatus::TASK_INITIALISED) {
        return parent;
    }
    duration = cfg.task_duration;
	counter = 0;
	debug("Initialising (duration = %d)...\n",duration);
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskLong::iterate()
{
	char tmp[128];
	while (counter < duration) {
		taskStatus = TaskStatus::TASK_RUNNING;
		sprintf(tmp,"Longing (%d/%.1f)...\n",counter,duration);
		setStatusString(tmp);
		debug(tmp);
		counter += 1;
		sleep(1);
	}
	debug(" -= Task completed =- \n");
	return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskLong::terminate()
{
	debug("Terminating...\n");
	return TaskStatus::TASK_TERMINATED;
}


DYNAMIC_TASK(TaskLong);
