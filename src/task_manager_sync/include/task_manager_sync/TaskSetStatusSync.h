#ifndef TASK_SET_STATUS_SYNC_H
#define TASK_SET_STATUS_SYNC_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_sync/TaskEnvironmentSync.h"

using namespace task_manager_lib;

namespace task_manager_sync {
    struct TaskSetStatusSyncConfig : public TaskConfig {
        TaskSetStatusSyncConfig() : TaskConfig() {
            define("status",0,"Status id. This should correspond to an enum.",true);
        }
    };


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
}

#endif // TASK_SET_STATUS_SYNC_H

