#ifndef TASK_TEST_H
#define TASK_TEST_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_test/TaskTestConfig.h"
using namespace task_manager_lib;

namespace task_manager_test {
    class TaskTest : public TaskDefinitionWithConfig<TaskTestConfig,TaskTest>
    {
        protected:
            double duration;
            unsigned int counter;
        public:
            TaskTest(boost::shared_ptr<TaskEnvironment> env) : Parent("Test","Do nothing",true,-1.), counter(0) {}
            virtual ~TaskTest() {};

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_TEST_H
