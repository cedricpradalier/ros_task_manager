#ifndef TASK_Constant_H
#define TASK_Constant_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskConstantConfig : public TaskConfig {
        TaskConstantConfig() {
            define("duration",  0.,"Duration of the action",false, duration);
            define("linear",  0.,"Linear velocity",false, linear);
            define("angular",  0.,"Angular velocity",false, angular);
        }

        // convenience aliases, updated by update from the config data
        double duration;
        double linear;
        double angular;
    };

    class TaskConstant : public TaskInstance<TaskConstantConfig,TurtleSimEnv>
    {
        protected:
            rclcpp::Time t0;
        public:
            TaskConstant(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskConstant() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryConstant : public TaskDefinition<TaskConstantConfig, TurtleSimEnv, TaskConstant>
    {

        public:
            TaskFactoryConstant(TaskEnvironmentPtr env) : 
                Parent("Constant","Apply a constant command for a given duration",true,env) {}
            virtual ~TaskFactoryConstant() {};
    };
};

#endif // TASK_Constant_H
