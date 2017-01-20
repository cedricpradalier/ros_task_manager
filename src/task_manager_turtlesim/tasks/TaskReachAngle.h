#ifndef TASK_REACH_ANGLE_H
#define TASK_REACH_ANGLE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskReachAngleConfig.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
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
};

#endif // TASK_REACH_ANGLE_H
