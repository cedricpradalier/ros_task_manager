#ifndef TASK_WAIT_FOR_STATUS_SYNC_H
#define TASK_WAIT_FOR_STATUS_SYNC_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_sync/TaskEnvironmentSync.h"

using namespace task_manager_lib;

namespace task_manager_sync {
    struct TaskWaitForStatusSyncConfig : public TaskConfig {
        TaskWaitForStatusSyncConfig() : TaskConfig() {
            define("partner","","Name of the synchronised object we're waiting for (as defined in env).",true);
            define("status",0,"Value of the status we're waiting for. Ideally, it should be an enum.",true);
        }
    };


    class TaskWaitForStatusSync : public TaskInstance<TaskWaitForStatusSyncConfig,TaskEnvironmentSync>
    {
        public:
            TaskWaitForStatusSync(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForStatusSync() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

    };

    class TaskFactoryWaitForStatusSync : public TaskDefinition<TaskWaitForStatusSyncConfig, TaskEnvironmentSync, TaskWaitForStatusSync>
    {

        public:
            TaskFactoryWaitForStatusSync(TaskEnvironmentPtr env) : 
                Parent("WaitForStatusSync","Wait for a sync object to get the desired status",true,env) {}
            virtual ~TaskFactoryWaitForStatusSync() {};
    };
}

#endif // TASK_WAIT_FOR_STATUS_SYNC_H

