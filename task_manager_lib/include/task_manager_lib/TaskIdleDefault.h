#ifndef TASK_IDLE_DEFAULT_H
#define TASK_IDLE_DEFAULT_H

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    // Basic class to implement the idle behaviour.
    // This task does not do anything.
    class TaskIdleDefault : public TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig,TaskIdleDefault>
    {
        protected:
        public:
            // Basic constructor, receives the environment and ignore it.
            // The class is periodic with whatever period the scheduler
            // prefers.
            TaskIdleDefault(boost::shared_ptr<TaskEnvironment> env) 
                : TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig,TaskIdleDefault>("Idle","Do nothing",true,-1) {}
            virtual ~TaskIdleDefault() {};

            // Rreturns immediately.
            virtual TaskIndicator iterate();

            // OK because there is no private data to this task
            virtual boost::shared_ptr<TaskDefinition> getInstance() {
                return shared_from_this();
            }

    };
};

#endif // TASK_IDLE_DEFAULT_H
