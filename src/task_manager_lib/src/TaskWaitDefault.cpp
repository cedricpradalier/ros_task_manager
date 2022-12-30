#include "task_manager_lib/TaskWaitDefault.h"
using namespace task_manager_msgs::msg;
using namespace task_manager_lib;

TaskIndicator TaskWaitDefault::initialise() 
{
    // ROS_INFO("TaskWait: initialised (%.2f)",cfg.duration);
    t0 = this->get_clock()->now();
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitDefault::iterate()
{
    rclcpp::Duration d = this->get_clock()->now() - t0;
    // ROS_INFO("TaskWait: waited %.2fs",d.toSec());
        
    if (d.seconds() > cfg->duration.get<double>()) {
        return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


// DYNAMIC_TASK(DefinitionWaitDefault);
