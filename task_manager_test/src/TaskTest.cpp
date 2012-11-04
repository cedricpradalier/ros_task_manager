
#include "task_manager_test/TaskTest.h"
#include "task_manager_test/TaskTestConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_test;

TaskIndicator TaskTest::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	debug("Configuring...\n");
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskTest::initialise(const TaskParameters & parameters) throw (InvalidParameter)
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

TaskIndicator TaskTest::iterate()
{
	debug("Testing (%d/%.1f)...\n",counter,duration);
	counter += 1;
	if (counter >= duration) {
		debug(" -= Task Completed =-\n");
		return TaskStatus::TASK_COMPLETED;
	}
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskTest::terminate()
{
	debug("Terminating...\n");
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskTest);
