#ifndef TASK_CLEAR_H
#define TASK_CLEAR_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskClear : public TaskInstance<TaskConfig,TurtleSimEnv>
    {
        protected:
            typedef enum {
                WAITING_FOR_CLIENT,
                WAITING_FOR_FUTURE
            } ClientState;
            ClientState state;
            rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future;
        public:
            TaskClear(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskClear() {};

            virtual TaskIndicator initialise();
            virtual TaskIndicator iterate();
    };
    class TaskFactoryClear : public TaskDefinition<TaskConfig, TurtleSimEnv, TaskClear>
    {

        public:
            TaskFactoryClear(TaskEnvironmentPtr env) : 
                Parent("Clear","Clear the screen",true,env) {}
            virtual ~TaskFactoryClear() {};
    };
}

#endif // TASK_CLEAR_H
