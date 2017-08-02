#ifndef TASK_SET_STATUS_SYNC_H
#define TASK_SET_STATUS_SYNC_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_sync/TaskEnvironmentSync.h"
#include "task_manager_sync/TaskSetStatusSyncConfig.h"

using namespace task_manager_lib;

namespace task_manager_sync {
    class TaskSetStatusSync : public TaskInstance<TaskSetStatusSyncConfig,TaskEnvironmentSync>
    {
        public:
            TaskSetStatusSync(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetStatusSync() {};

            virtual TaskIndicator iterate();
    };
    class TaskFactorySetStatusSync : public TaskDefinition<TaskSetStatusSyncConfig, TaskEnvironmentSync, TaskSetStatusSync>
    {

        public:
            TaskFactorySetStatusSync(TaskEnvironmentPtr env) : 
                Parent("SetStatusSync","Update the current task status to be published by the environment",true,env) {}
            virtual ~TaskFactorySetStatusSync() {};
    };
};

#endif // TASK_SET_STATUS_SYNC_H

