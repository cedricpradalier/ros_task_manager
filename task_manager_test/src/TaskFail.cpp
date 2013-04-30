
#include "task_manager_test/TaskFail.h"
#include "task_manager_test/TaskFailConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_test;

TaskIndicator TaskFail::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskFail::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator parent = Parent::initialise(parameters);
    if (parent != TaskStatus::TASK_INITIALISED) {
        return parent;
    }
    duration = cfg.task_duration;
	counter = 0;
	debug("Initialising (duration = %.1f)...\n",duration);
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskFail::iterate()
{
	debug("Testing (%d/%.1f)...\n",counter,duration);
	counter += 1;
	if (counter >= duration) {
		debug(" -= Task FAILED =-\n");
		return TaskStatus::TASK_FAILED;
	}
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFail::terminate()
{
	debug("Terminating...\n");
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFail);
