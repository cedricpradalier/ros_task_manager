#ifndef TASK_SYSTEM_H
#define TASK_SYSTEM_H

#include "task_manager_lib/TaskInstance.h"

namespace task_manager_lib {
    struct TaskSystemConfig : public TaskConfig {
        double terminate_time;
        std::string command;
        std::vector<std::string> command_array;
        TaskSystemConfig() {
            define("terminate_time",5.0,"Tolerance time to terminate (s)",true,terminate_time); 
            define("command",std::string("exit"),"Command to execute",true,command); 
            define("command_array",command_array,"Command to execute in an array form (if it is not empty, 'command' will be ignored)",true,command_array); 
        }
    };

    class TaskSystem : public TaskInstance<TaskSystemConfig,TaskEnvironment>
    {
        protected:
            pid_t pid;
        public:
            // Basic constructor, receives the environment and ignore it.
            TaskSystem(TaskDefinitionPtr def, TaskEnvironmentPtr ev) :
                Parent(def,ev) {}
            virtual ~TaskSystem() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();


    };

    class TaskFactorySystem : public TaskDefinition<TaskSystemConfig,TaskEnvironment, TaskSystem>
    {
        public:
            TaskFactorySystem(TaskEnvironmentPtr env) 
                : Parent("System","Execute a system command",true,env) {}
            virtual ~TaskFactorySystem() {};

    };

};

#endif // TASK_SYSTEM_H
