#include "task_manager_sync/TaskWaitForStatusSync.h"
using namespace task_manager_lib;
using namespace task_manager_sync;


TaskIndicator TaskWaitForStatusSync::initialise() 
{
    int cfg_status = cfg->get<int>("status");
    const std::string & partner = cfg->get<std::string>("partner");
    if (env->isStatusValidForSource(partner,cfg_status)) {
        RCLCPP_INFO(node->get_logger(),"Waiting for partner '%s' to reach status %d",partner.c_str(),cfg_status);
        return TaskStatus::TASK_INITIALISED;
    } else {
        RCLCPP_ERROR(node->get_logger(),"Status %d is not a valid status for partner '%s'",cfg_status,partner.c_str());
        return TaskStatus::TASK_INITIALISATION_FAILED;
    }
}


TaskIndicator TaskWaitForStatusSync::iterate()
{
    int cfg_status = cfg->get<int>("status");
    const std::string & partner = cfg->get<std::string>("partner");
    int status = 0;
    if (!env->getStatusRef(partner,status)) {
        // not yet received
        return TaskStatus::TASK_RUNNING;
    }
    if (status == cfg_status)    {
        return TaskStatus::TASK_COMPLETED;
    }
    return TaskStatus::TASK_RUNNING;
}

// This is not designed to be a dynamic task. Use TaskServerSync instead
// DYNAMIC_TASK(TaskFactoryWaitForStatusSync);

