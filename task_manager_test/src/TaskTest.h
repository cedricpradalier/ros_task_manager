#ifndef TASK_TEST_H
#define TASK_TEST_H

#include "TaskDefinition.h"

class TaskTest : public TaskDefinition
{
	protected:
		unsigned int duration;
		unsigned int counter;
	public:
		TaskTest(TaskEnvironment *env) : TaskDefinition("Test","Do nothing",true,5.0), counter(0) {}
		virtual ~TaskTest() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_TEST_H
