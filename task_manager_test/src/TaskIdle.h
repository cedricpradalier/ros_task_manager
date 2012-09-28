#ifndef TASK_IDLE_H
#define TASK_IDLE_H

#include "TaskDefinition.h"

class TaskIdle : public TaskDefinition
{
	protected:
	public:
		TaskIdle(TaskEnvironment *env) : TaskDefinition("Idle","Do nothing",true,-1) {}
		virtual ~TaskIdle() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_IDLE_H
