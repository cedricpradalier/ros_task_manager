#ifndef TASK_SETPEN_H
#define TASK_SETPEN_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"

namespace task_manager_turtlesim {
    class TaskSetPen : public TaskDefinitionWithConfig<TaskSetPenConfig>
    {
        protected:
            boost::shared_ptr<TurtleSimEnv> env;
            TaskSetPenConfig cfg;
        public:
            TaskSetPen(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskSetPen() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_SETPEN_H
