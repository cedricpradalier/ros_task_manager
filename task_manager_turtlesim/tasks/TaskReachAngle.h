#ifndef TASK_REACH_ANGLE_H
#define TASK_REACH_ANGLE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskReachAngleConfig.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskReachAngle : public TaskDefinitionWithConfig<TaskReachAngleConfig,TaskReachAngle>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
        public:
            TaskReachAngle(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskReachAngle() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_REACH_ANGLE_H
