#include <math.h>
#include "TaskFail.h"
#include "task_manager_turtlesim/TaskFailConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskIndicator TaskFactoryFail::configure(const TaskParameters & parameters) throw (InvalidParameter) {
    TaskIndicator ti = Parent::configure(parameters);
    if (ti != TaskStatus::TASK_CONFIGURED) {
        return ti;
    }
    TaskFailConfig cfg = getConfigFromServer(env->getNodeHandle());
    if (cfg.error_type == TaskStatus::TASK_CONFIGURATION_FAILED) {
        return cfg.error_type;
    }
    return ti;
}

TaskIndicator TaskFail::initialise()  {
    counter = 0;
    if (cfg.error_type == TaskStatus::TASK_INITIALISATION_FAILED) {
        return cfg.error_type;
    }
    return TaskStatus::TASK_INITIALISED;
}
            

TaskIndicator TaskFail::iterate()
{
    if ((signed)counter >= cfg.iterations) {
        return cfg.error_type;
    }
    counter += 1;
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFail::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryFail);
