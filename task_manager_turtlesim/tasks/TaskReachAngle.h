#ifndef TASK_REACH_ANGLE_H
#define TASK_REACH_ANGLE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskReachAngleConfig.h"

namespace task_manager_turtlesim {
    class TaskReachAngle : public TaskDefinitionWithConfig<TaskReachAngleConfig>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
            TaskReachAngleConfig cfg;
        public:
            TaskReachAngle(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskReachAngle() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_REACH_ANGLE_H
