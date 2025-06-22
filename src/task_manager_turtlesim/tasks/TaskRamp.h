#ifndef TASK_Ramp_H
#define TASK_Ramp_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskRampConfig : public TaskConfig {
        TaskRampConfig() {
            define("duration",  0.,"Duration of the action",false, duration);
            define("linear_min",  0.,"Linear velocity at start",false, linear_min);
            define("linear_max",  0.,"Linear velocity at end",false, linear_max);
            define("angular_min",  0.,"Angular velocity at start",false, angular_min);
            define("angular_max",  0.,"Angular velocity at end",false, angular_max);
        }

        // convenience aliases, updated by update from the config data
        double duration;
        double linear_min,linear_max;
        double angular_min,angular_max;
    };

    class TaskRamp : public TaskInstance<TaskRampConfig,TurtleSimEnv>
    {
        protected:
            rclcpp::Time t0;
        public:
            TaskRamp(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskRamp() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryRamp : public TaskDefinition<TaskRampConfig, TurtleSimEnv, TaskRamp>
    {

        public:
            TaskFactoryRamp(TaskEnvironmentPtr env) : 
                Parent("Ramp","Apply a Ramp command for a given duration",true,env) {}
            virtual ~TaskFactoryRamp() {};
    };
};

#endif // TASK_Ramp_H
