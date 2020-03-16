#ifndef TASK_FollowPath_H
#define TASK_FollowPath_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskFollowPathConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    
    class TaskFollowPath : public TaskInstance<TaskFollowPathConfig, TurtleSimEnv>
    {
        protected:
            turtlesim::Pose initial_pose;

        public:
            TaskFollowPath(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskFollowPath() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryFollowPath : public TaskDefinition<TaskFollowPathConfig, TurtleSimEnv, TaskFollowPath>
    {

        public:
            TaskFactoryFollowPath(TaskEnvironmentPtr env) : 
                Parent("FollowPath","Follow a list of way point",true,env) {}
            virtual ~TaskFactoryFollowPath() {};
    };
};

#endif // TASK_FollowPath_H
