#include <math.h>
#include "TaskSetPen.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskSetPen::TaskSetPen(boost::shared_ptr<TaskEnvironment> tenv)
    : TaskDefinitionWithConfig<TaskSetPenConfig>("SetPen","Set pen value",false,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskSetPen::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskSetPen::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    cfg = parameters.toConfig<TaskSetPenConfig>();
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskSetPen::iterate()
{
    env->setPen(cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    printf("Set pen to %d %d %d %d %d\n",cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskSetPen::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskSetPen);
