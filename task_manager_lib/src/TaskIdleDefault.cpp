#include "task_manager_lib/TaskIdleDefault.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;

TaskIndicator TaskIdleDefault::iterate()
{
	return TaskStatus::TASK_RUNNING;
}



// DYNAMIC_TASK(DefinitionIdleDefault);
