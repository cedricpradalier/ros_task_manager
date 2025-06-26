#ifndef TASK_Trigger_H
#define TASK_Trigger_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_lib/TaskServiceGeneric.h"
#include "std_srvs/srv/trigger.hpp"

namespace task_manager_lib {
    struct TaskTriggerConfig : public TaskServiceGenericConfig {
        TaskTriggerConfig() : TaskServiceGenericConfig("/service_name") {
        }
    };

    class TaskTrigger : public TaskServiceGeneric<std_srvs::srv::Trigger,TaskTriggerConfig,TaskEnvironment>
    {
        protected:

        public:
            TaskTrigger(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskTrigger() {};
    };

    class TaskFactoryTrigger : public TaskDefinition<TaskTriggerConfig, TaskEnvironment, TaskTrigger>
    {

        public:
            TaskFactoryTrigger(TaskEnvironmentPtr env) : 
                Parent("Trigger","Use a service to send a trigger",true,env) {}
            virtual ~TaskFactoryTrigger() {};
    };
}

#endif // TASK_Trigger_H
