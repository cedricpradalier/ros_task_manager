#ifndef TASK_CLEAR_H
#define TASK_CLEAR_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskClear : public TaskServiceGenericWithoutClient<std_srvs::srv::Empty,
    TaskServiceGenericWithoutClientConfig,TurtleSimEnv>
    {
        protected:
            virtual rclcpp::Client<std_srvs::srv::Empty>::SharedPtr getServiceClient() {
                return env->getClearClient();
            }
        public:
            TaskClear(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskClear() {};
    };

    class TaskFactoryClear : public TaskDefinition<TaskServiceGenericWithoutClientConfig, TurtleSimEnv, TaskClear>
    {

        public:
            TaskFactoryClear(TaskEnvironmentPtr env) : 
                Parent("ClearGenericWC","Clear the screen (GenericWC)",true,env) {}
            virtual ~TaskFactoryClear() {};
    };
}

#endif // TASK_CLEAR_H
