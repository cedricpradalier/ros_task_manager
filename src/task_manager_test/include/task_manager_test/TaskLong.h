#ifndef TASK_LONG_H
#define TASK_LONG_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_test/TaskLongConfig.h"
using namespace task_manager_lib;

namespace task_manager_test {
    class TaskLong : public TaskDefinitionWithConfig<TaskLongConfig,TaskLong>
    {
        protected:
            double duration;
            unsigned int counter;
        public:
            TaskLong(boost::shared_ptr<TaskEnvironment> env) : Parent("Long","Wait longly",false,25.0), counter(0) {}
            virtual ~TaskLong() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_LONG_H
