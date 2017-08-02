#ifndef TASK_WAIT_FOR_STATUS_SYNC_H
#define TASK_WAIT_FOR_STATUS_SYNC_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_sync/TaskEnvironmentSync.h"
#include "task_manager_sync/TaskWaitForStatusSyncConfig.h"

using namespace task_manager_lib;

namespace task_manager_sync {
    class TaskWaitForStatusSync : public TaskInstance<TaskWaitForStatusSyncConfig,TaskEnvironmentSync>
    {
        public:
            TaskWaitForStatusSync(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForStatusSync() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryWaitForStatusSync : public TaskDefinition<TaskWaitForStatusSyncConfig, TaskEnvironmentSync, TaskWaitForStatusSync>
    {

        public:
            TaskFactoryWaitForStatusSync(TaskEnvironmentPtr env) : 
                Parent("WaitForStatusSync","Wait for a sync object to get the desired status",true,env) {}
            virtual ~TaskFactoryWaitForStatusSync() {};
    };
};

#endif // TASK_WAIT_FOR_STATUS_SYNC_H

