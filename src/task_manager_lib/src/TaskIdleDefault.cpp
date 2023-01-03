#include "task_manager_lib/TaskIdleDefault.h"
using namespace task_manager_msgs::msg;
using namespace task_manager_lib;

TaskIndicator TaskIdleDefault::iterate()
{
    // RCLCPP_INFO(node->get_logger(),"Idling...");
	return TaskStatus::TASK_RUNNING;
}



// DYNAMIC_TASK(DefinitionIdleDefault);
