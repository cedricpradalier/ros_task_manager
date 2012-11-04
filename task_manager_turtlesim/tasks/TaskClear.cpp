#include <math.h>
#include "TaskClear.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskClear::TaskClear(boost::shared_ptr<TaskEnvironment> tenv)
    : Parent("Clear","Clear the screen",false,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}


TaskIndicator TaskClear::iterate()
{
    env->clear();
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskClear);
