#include <math.h>
#include "TaskRamp.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskRamp::initialise() 
{
    t0 = getNode()->get_clock()->now();
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskRamp::iterate()
{
    double dt = (getNode()->get_clock()->now()-t0).seconds();
    if (dt > cfg->duration) {
        return TaskStatus::TASK_COMPLETED;
    }
    env->publishVelocity( cfg->linear_min +
            dt * (cfg->linear_max-cfg->linear_min)/cfg->duration, 
            cfg->angular_min+
            dt * (cfg->angular_max-cfg->angular_min)/cfg->duration);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskRamp::terminate()
{
    env->publishVelocity(cfg->linear_max,cfg->angular_max);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryRamp);
