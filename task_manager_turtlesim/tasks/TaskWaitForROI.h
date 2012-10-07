#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskWaitForROIConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskWaitForROI : public TaskDefinitionWithConfig<TaskWaitForROIConfig>
    {

        protected:
            boost::shared_ptr<TurtleSimEnv> env;
            TaskWaitForROIConfig cfg;
        public:
            TaskWaitForROI(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskWaitForROI() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_WAIT_FOR_ROI_H
