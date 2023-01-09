#ifndef TASK_CLEAR_H
#define TASK_CLEAR_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskClearConfig : public TaskServiceGenericConfig {
        TaskClearConfig() : TaskServiceGenericConfig("/clear") {
        }
    };

    class TaskClear : public TaskServiceGeneric<std_srvs::srv::Empty,TaskClearConfig,TurtleSimEnv>
    {
        protected:
        public:
            TaskClear(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskClear() {};
    };

    class TaskFactoryClear : public TaskDefinition<TaskClearConfig, TurtleSimEnv, TaskClear>
    {

        public:
            TaskFactoryClear(TaskEnvironmentPtr env) : 
                Parent("ClearGeneric","Clear the screen (Generic)",true,env) {}
            virtual ~TaskFactoryClear() {};
    };
}

#endif // TASK_CLEAR_H
