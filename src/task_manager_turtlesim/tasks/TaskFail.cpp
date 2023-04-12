#include <math.h>
#include "TaskFail.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskIndicator TaskFail::initialise()  {
    counter = 0;
    if (cfg->error_type == TaskStatus::TASK_INITIALISATION_FAILED) {
        return TaskStatus::TASK_INITIALISATION_FAILED;
    }
    return TaskStatus::TASK_INITIALISED;
}
            

TaskIndicator TaskFail::iterate()
{
    int error = cfg->error_type;
    if ((signed)counter >= cfg->iterations) {
        return error;
    }
    counter += 1;
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFail::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryFail)
