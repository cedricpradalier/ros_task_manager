#ifndef TASK_IDLE_DEFAULT_H
#define TASK_IDLE_DEFAULT_H

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {

    // Basic class to implement the idle behaviour.
    // This task does not do anything.

    class TaskIdleDefault : public TaskInstance<MinimalTaskConfig,TaskEnvironment>
    {
        protected:
        public:
            TaskIdleDefault(TaskDefinitionPtr def, TaskEnvironmentPtr ev) :
                Parent(def,ev) {}
            virtual ~TaskIdleDefault() {}

            // Returns immediately.
            virtual TaskIndicator iterate();

    };

    class TaskFactoryIdleDefault : public TaskDefinition<MinimalTaskConfig,TaskEnvironment,TaskIdleDefault>
    {
        protected:
        public:
            // Basic constructor, receives the environment and ignore it.
            // The class is periodic with whatever period the scheduler
            // prefers.
            TaskFactoryIdleDefault(TaskEnvironmentPtr env) 
                : Parent("Idle","Do nothing",true,env) {}
            virtual ~TaskFactoryIdleDefault() {};
    };

};

#endif // TASK_IDLE_DEFAULT_H
