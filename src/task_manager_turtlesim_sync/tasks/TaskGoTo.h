#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include <turtlesim/msg/pose.hpp>
#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim_sync/TurtleSimEnv.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim_sync {
    struct TaskGoToConfig : public TaskConfig {
        TaskGoToConfig() {
            define("goal_x",  0.,"X coordinate of destination",false);
            define("goal_y",  0.,"Y coordinate of destination",false);
            define("k_v",  1.0,"Gain for velocity control",false);
            define("k_alpha",  1.0,"Gain for angular control",false);
            define("max_velocity",  1.0,"Max allowed velocity",false);
            define("dist_threshold",  0.1,"Distance at which a the target is considered reached",false);
            define("relative",  false,"Is the target pose relative or absolute",true);
        }

        void update();

        // convenience aliases, updated by update from the config data
        double goal_x,goal_y;
        double k_v,k_alpha;
        double max_velocity;
        double dist_threshold;
        bool relative;

    };

    class TaskGoTo : public TaskInstance<TaskGoToConfig, TurtleSimEnv>
    {
        protected:
            turtlesim::msg::Pose initial_pose;

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
}

#endif // TASK_GOTO_H
