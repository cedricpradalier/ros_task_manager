#include <math.h>
#include "TaskClear.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskClear::iterate()
{
    std_msgs::Header header;
    if (extractEncapsulatedMessage(header)) {
        ROS_INFO("Clear: got an extra header, frame_id = %s",header.frame_id.c_str());
    }
    env->clear();
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskFactoryClear);
