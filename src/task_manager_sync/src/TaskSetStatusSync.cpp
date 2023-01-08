
#include "task_manager_sync/TaskSetStatusSync.h"
using namespace task_manager_lib;
using namespace task_manager_sync;



TaskIndicator TaskSetStatusSync::iterate()
{
    int cfg_status = cfg->get<int>("status");
    if (env->isStatusValidForMe(cfg_status)) {
        env->setStatus(cfg_status);
        return TaskStatus::TASK_COMPLETED;
    } else {
        RCLCPP_ERROR(node->get_logger(),"TaskSetStatusSync: Invalid status request %d",cfg_status);
        return TaskStatus::TASK_COMPLETED;
    }
}



// This is not designed to be a dynamic task. Use TaskServerSync instead
// DYNAMIC_TASK(TaskFactorySetStatusSync);

