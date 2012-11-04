#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskGoToConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskGoTo : public TaskDefinitionWithConfig<TaskGoToConfig, TaskGoTo>
    {

        protected:
            boost::shared_ptr<TurtleSimEnv> env;
        public:
            TaskGoTo(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskGoTo() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_GOTO_H
