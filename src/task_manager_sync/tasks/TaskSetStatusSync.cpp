
#include <math.h>
#include "task_manager_sync/TaskSetStatusSync.h"
#include "task_manager_sync/TaskSetStatusSyncConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_sync;



TaskIndicator TaskSetStatusSync::iterate()
{
    if (env->isStatusValidForMe(cfg.status)) {
        env->setStatus(cfg.status);
        return TaskStatus::TASK_COMPLETED;
    } else {
        ROS_ERROR("TaskSetStatusSync: Invalid status request %d",cfg.status);
        return TaskStatus::TASK_COMPLETED;
    }
}



// This is not designed to be a dynamic task. Use TaskServerSync instead
// DYNAMIC_TASK(TaskFactorySetStatusSync);

