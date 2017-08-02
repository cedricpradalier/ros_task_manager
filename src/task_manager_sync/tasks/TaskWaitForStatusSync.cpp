#include <math.h>
#include "task_manager_sync/TaskWaitForStatusSync.h"
#include "task_manager_sync/TaskWaitForStatusSyncConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_sync;


TaskIndicator TaskWaitForStatusSync::initialise() 
{
    if (env->isStatusValidForSource(cfg.partner,cfg.status)) {
        ROS_INFO("Waiting for partner '%s' to reach status %d",cfg.partner.c_str(),cfg.status);
        return TaskStatus::TASK_INITIALISED;
    } else {
        ROS_ERROR("Status %d is not a valid status for partner '%s'",cfg.status,cfg.partner.c_str());
        return TaskStatus::TASK_INITIALISATION_FAILED;
    }
}


TaskIndicator TaskWaitForStatusSync::iterate()
{
    int status = 0;
    if (!env->getStatusRef(cfg.partner,status)) {
        // not yet received
        return TaskStatus::TASK_RUNNING;
    }
    if (status == cfg.status)    {
        return TaskStatus::TASK_COMPLETED;
    }
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForStatusSync::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

// This is not designed to be a dynamic task. Use TaskServerSync instead
// DYNAMIC_TASK(TaskFactoryWaitForStatusSync);

