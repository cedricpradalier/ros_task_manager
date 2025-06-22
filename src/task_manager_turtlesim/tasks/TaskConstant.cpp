#include <math.h>
#include "TaskConstant.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskConstant::initialise() 
{
    t0 = getNode()->get_clock()->now();
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskConstant::iterate()
{
    if ((getNode()->get_clock()->now()-t0).seconds() > cfg->duration) {
        return TaskStatus::TASK_COMPLETED;
    }
    env->publishVelocity(cfg->linear, cfg->angular);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskConstant::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryConstant);
