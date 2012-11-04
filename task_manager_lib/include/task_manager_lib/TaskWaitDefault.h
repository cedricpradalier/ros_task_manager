#ifndef TASK_WAIT_DEFAULT_H
#define TASK_WAIT_DEFAULT_H

#include "task_manager_lib/TaskWaitConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    // Basic class to implement a short pause in a mission 
    // This task does not do anything apart from waiting.
    // It still has a .cfg file in cfg/TaskWait.cfg which describe the duration
    // parameter
    class TaskWaitDefault : public TaskDefinitionWithConfig<task_manager_lib::TaskWaitConfig,TaskWaitDefault>
    {
        protected:
            TaskWaitConfig cfg;
            ros::Time t0;
        public:
            // Basic constructor, receives the environment and ignore it.
            // The class is periodic with whatever period the scheduler
            // prefers.
            TaskWaitDefault(boost::shared_ptr<TaskEnvironment> env) 
                : TaskDefinitionWithConfig<task_manager_lib::TaskWaitConfig,TaskWaitDefault>("Wait","Do nothing for a given time",true,-1) {}
            virtual ~TaskWaitDefault() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            /// Record the starting time
            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            // Return COMPLETED once the current time - starting time is larger than the
            // desired duration
            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

            // We need to make a copy to make sure the private data stays
            // private
            virtual boost::shared_ptr<TaskDefinition> getInstance() {
                return boost::shared_ptr<TaskDefinition>(new TaskWaitDefault(*this));
            }

    };
};

#endif // TASK_WAIT_DEFAULT_H
