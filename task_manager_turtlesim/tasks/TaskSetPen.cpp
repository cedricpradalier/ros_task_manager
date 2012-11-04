#include <math.h>
#include "TaskSetPen.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskSetPen::TaskSetPen(boost::shared_ptr<TaskEnvironment> tenv)
    : Parent("SetPen","Set pen value",false,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskSetPen::iterate()
{
    env->setPen(cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    printf("Set pen to %d %d %d %d %d\n",cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskSetPen);
