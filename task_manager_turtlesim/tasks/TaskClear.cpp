#include <math.h>
#include "TaskClear.h"
using namespace task_manager_msgs;
using namespace task_manager_turtlesim;

TaskClear::TaskClear(boost::shared_ptr<TaskEnvironment> tenv)
    : TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig>("Clear","Clear the screen",false,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskClear::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskClear::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskClear::iterate()
{
    env->clear();
    return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskClear::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskClear);
