#ifndef TASK_CLEAR_H
#define TASK_CLEAR_H

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskClear : public TaskInstance<MinimalTaskConfig,TurtleSimEnv>
    {
        public:
            TaskClear(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskClear() {};

            virtual TaskIndicator iterate();
    };
    class TaskFactoryClear : public TaskDefinition<MinimalTaskConfig, TurtleSimEnv, TaskClear>
    {

        public:
            TaskFactoryClear(TaskEnvironmentPtr env) : 
                Parent("Clear","Clear the screen",false,env) {}
            virtual ~TaskFactoryClear() {};
    };
};

#endif // TASK_CLEAR_H
