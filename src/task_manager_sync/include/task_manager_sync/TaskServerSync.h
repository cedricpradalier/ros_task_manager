#ifndef TASK_SERVER_SYNC_H
#define TASK_SERVER_SYNC_H

#include <rclcpp/rclcpp.hpp>
#include <task_manager_lib/TaskDefinition.h>
#include <task_manager_lib/TaskServerDefault.h>
#include <task_manager_sync/TaskSetStatusSync.h>
#include <task_manager_sync/TaskWaitForStatusSync.h>


namespace task_manager_sync {
    class TaskServerSync : public task_manager_lib::TaskServerBase {
        public:
            TaskServerSync(TaskEnvironmentPtr _env, bool default_wait=true);  
    };

}

#endif // TASK_SERVER_SYNC_H

