#ifndef TASK_IDLE_H
#define TASK_IDLE_H

#include "TaskDefinition.h"
#include "TaskCoaxEnvironment.h"

class TaskIdle : public TaskDefinition
{
	protected:
		SBApiSimpleContext *interface;
	public:
		TaskIdle(TaskEnvironment *env);
		virtual ~TaskIdle() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_IDLE_H
