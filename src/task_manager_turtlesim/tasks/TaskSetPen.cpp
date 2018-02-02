#include <math.h>
#include "TaskSetPen.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskIndicator TaskSetPen::iterate()
{
    ROS_INFO("Set pen to %d %d %d %d %d",cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    env->setPen(cfg.on,cfg.r,cfg.g,cfg.b,cfg.width);
    ros::Duration(cfg.artificial_delay).sleep();
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskFactorySetPen);
