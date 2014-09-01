#ifndef TASK_FAIL_H
#define TASK_FAIL_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskFailConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskFail : public TaskInstance<TaskFailConfig, TurtleSimEnv>
    {
        protected:
            unsigned int counter;
        public:
            TaskFail(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskFail() {};

            virtual TaskIndicator initialise() ;
            
            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };

    class TaskFactoryFail : public TaskDefinition<TaskFailConfig, TurtleSimEnv, TaskFail>
    {

        public:
            TaskFactoryFail(TaskEnvironmentPtr env) : 
                Parent("Fail","Fail with a specified condition",true,env) {}
            virtual ~TaskFactoryFail() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);
    };
};

#endif // TASK_FAIL_H
