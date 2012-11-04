#ifndef TASK_SETPEN_H
#define TASK_SETPEN_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskSetPen : public TaskDefinitionWithConfig<TaskSetPenConfig,TaskSetPen>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
        public:
            TaskSetPen(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskSetPen() {};

            virtual TaskIndicator iterate();

    };
};

#endif // TASK_SETPEN_H
