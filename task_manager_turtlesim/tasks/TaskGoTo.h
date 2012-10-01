#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskGoToConfig.h"

namespace task_manager_turtlesim {
    class TaskGoTo : public TaskDefinitionWithConfig<TaskGoToConfig>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
            TaskGoToConfig cfg;
        public:
            TaskGoTo(boost::shared_ptr<TaskEnvironment> &env); 
            virtual ~TaskGoTo() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_GOTO_H
