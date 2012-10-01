#ifndef TASK_IDLE_DEFAULT_H
#define TASK_IDLE_DEFAULT_H

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    class TaskIdleDefault : public TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig>
    {
        protected:
        public:
            TaskIdleDefault(boost::shared_ptr<TaskEnvironment> &env) 
                : TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig>("Idle","Do nothing",true,-1) {}
            virtual ~TaskIdleDefault() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_IDLE_DEFAULT_H
