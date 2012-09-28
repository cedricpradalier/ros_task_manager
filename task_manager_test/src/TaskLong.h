#ifndef TASK_LONG_H
#define TASK_LONG_H

#include "TaskDefinition.h"

class TaskLong : public TaskDefinition
{
	protected:
		unsigned int duration;
		unsigned int counter;
	public:
		TaskLong(TaskEnvironment *env) : TaskDefinition("Long","Wait longly",false,25.0), counter(0) {}
		virtual ~TaskLong() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_LONG_H
