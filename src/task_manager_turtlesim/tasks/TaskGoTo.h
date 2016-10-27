#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskGoToConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskGoTo : public TaskInstance<TaskGoToConfig, TurtleSimEnv>
    {
        protected:
            turtlesim::Pose initial_pose;

        public:
            TaskGoTo(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoTo() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoTo : public TaskDefinition<TaskGoToConfig, TurtleSimEnv, TaskGoTo>
    {

        public:
            TaskFactoryGoTo(TaskEnvironmentPtr env) : 
                Parent("GoTo","Reach a desired destination",true,env) {}
            virtual ~TaskFactoryGoTo() {};
    };
};

#endif // TASK_GOTO_H
