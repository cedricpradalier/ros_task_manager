#ifndef TASK_CLEAR_H
#define TASK_CLEAR_H

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"

namespace task_manager_turtlesim {
    class TaskClear : public TaskDefinitionWithConfig<task_manager_lib::MinimalTaskConfig>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
        public:
            TaskClear(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskClear() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_CLEAR_H
