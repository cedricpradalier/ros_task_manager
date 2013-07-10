#ifndef TASK_FAIL_H
#define TASK_FAIL_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskFailConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskFail : public TaskDefinitionWithConfig<TaskFailConfig, TaskFail>
    {

        protected:
            boost::shared_ptr<TurtleSimEnv> env;
            unsigned int counter;
        public:
            TaskFail(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskFail() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);
            
            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_FAIL_H
