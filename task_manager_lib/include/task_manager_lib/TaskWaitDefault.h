#ifndef TASK_WAIT_DEFAULT_H
#define TASK_WAIT_DEFAULT_H

#include "task_manager_lib/TaskWaitConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    class TaskWaitDefault : public TaskDefinitionWithConfig<task_manager_lib::TaskWaitConfig>
    {
        protected:
            TaskWaitConfig cfg;
            ros::Time t0;
        public:
            TaskWaitDefault(boost::shared_ptr<TaskEnvironment> &env) 
                : TaskDefinitionWithConfig<task_manager_lib::TaskWaitConfig>("Wait","Do nothing for a given time",true,-1) {}
            virtual ~TaskWaitDefault() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_WAIT_DEFAULT_H
