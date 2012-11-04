#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskWaitForROIConfig.h"

using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskWaitForROI : public TaskDefinitionWithConfig<TaskWaitForROIConfig,TaskWaitForROI>
    {

        protected:
            boost::shared_ptr<TurtleSimEnv> env;
        public:
            TaskWaitForROI(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskWaitForROI() {};

            virtual TaskIndicator iterate();

    };
};

#endif // TASK_WAIT_FOR_ROI_H
