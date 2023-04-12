#ifndef TASK_WAIT_DEFAULT_H
#define TASK_WAIT_DEFAULT_H

#include "task_manager_lib/TaskInstance.h"

namespace task_manager_lib {

    struct TaskWaitConfig : public TaskConfig {
        double duration;
        TaskWaitConfig() {
            define("duration",1.0,"How long to wait (s)",true,duration); 
        }
    };

    // Basic class to implement a short pause in a mission 
    // This task does not do anything apart from waiting.
    // It still has a .cfg file in cfg/TaskWait.cfg which describe the duration
    // parameter
    class TaskWaitDefault : public TaskInstance<TaskWaitConfig,TaskEnvironment>
    {
        protected:
            rclcpp::Time t0;
        public:
            // Basic constructor, receives the environment and ignore it.
            TaskWaitDefault(TaskDefinitionPtr def, TaskEnvironmentPtr ev) :
                Parent(def,ev) {
            }
            virtual ~TaskWaitDefault() {};

            /// Record the starting time
            virtual TaskIndicator initialise() ;

            // Return COMPLETED once the current time - starting time is larger than the
            // desired duration
            virtual TaskIndicator iterate();


    };

    class TaskFactoryWaitDefault : public TaskDefinition<TaskWaitConfig,TaskEnvironment, TaskWaitDefault>
    {
        public:
            // Basic constructor, 
            // The class is periodic with whatever period the scheduler
            // prefers.
            TaskFactoryWaitDefault(TaskEnvironmentPtr env) 
                : Parent("Wait","Do nothing for a given time",true,env) {
            }
            virtual ~TaskFactoryWaitDefault() {};

    };

}

#endif // TASK_WAIT_DEFAULT_H
