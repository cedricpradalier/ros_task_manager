#include <math.h>
#include "TaskClear.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskClear::iterate()
{
    env->clear();
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskFactoryClear);
