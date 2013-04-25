#ifndef TASK_FAIL_H
#define TASK_FAIL_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_test/TaskFailConfig.h"
using namespace task_manager_lib;

namespace task_manager_test {
    class TaskFail : public TaskDefinitionWithConfig<TaskFailConfig,TaskFail>
    {
        protected:
            double duration;
            unsigned int counter;
        public:
            TaskFail(boost::shared_ptr<TaskEnvironment> env) : Parent("Fail","Do nothing and fail",true,-1.), counter(0) {}
            virtual ~TaskFail() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_FAIL_H
