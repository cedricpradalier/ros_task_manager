#ifndef TASK_SYSTEM_H
#define TASK_SYSTEM_H

#include "task_manager_lib/TaskSystemConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    // Basic class to implement a short pause in a mission 
    // This task does not do anything apart from waiting.
    // It still has a .cfg file in cfg/TaskWait.cfg which describe the duration
    // parameter
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
