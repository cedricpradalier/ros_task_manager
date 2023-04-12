#ifndef TASK_FAIL_H
#define TASK_FAIL_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskFailConfig : public TaskConfig {
        TaskFailConfig() {
            define("error_type",7,"Type of failure case to generate",  true, error_type);
            define("iterations",0,"Number of iteration before failing",  false, iterations);
        }
        int error_type;
        int iterations;
    };


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

    };
}

#endif // TASK_FAIL_H
