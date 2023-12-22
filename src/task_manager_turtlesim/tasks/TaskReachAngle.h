#ifndef TASK_REACH_ANGLE_H
#define TASK_REACH_ANGLE_H

#include <turtlesim/msg/pose.hpp>
#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskReachAngleConfig : public TaskConfig {
        TaskReachAngleConfig() {
            define("target",  0.,"Target angle",false, target);
            define("max_vrot",  1.0,"Maximum angular velocity",false,max_vrot);
            define("k_alpha",  1.0,"Gain for angular control",false,k_alpha);
            define("threshold",  0.01,"Angular error at which the target is considered reached",false, threshold);
            define("relative",  false,"Is the target pose relative or absolute",true, relative);
        }

        // convenience aliases, updated by update from the config data
        double target;
        double k_alpha;
        double max_vrot;
        double threshold;
        bool relative;

    };

    class TaskReachAngle : public TaskInstance<TaskReachAngleConfig,TurtleSimEnv>
    {
        protected:
            double initial_theta;
        public:
            TaskReachAngle(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskReachAngle() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryReachAngle : public TaskDefinition<TaskReachAngleConfig, TurtleSimEnv, TaskReachAngle>
    {
        public:
            TaskFactoryReachAngle(TaskEnvironmentPtr env) : 
                Parent("ReachAngle","Reach a desired angular set-point",true,env) {}
            virtual ~TaskFactoryReachAngle() {};
    };
}

#endif // TASK_REACH_ANGLE_H
