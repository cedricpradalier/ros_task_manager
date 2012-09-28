#ifndef TASK_TAKEOFF_H
#define TASK_TAKEOFF_H

#include "TaskDefinition.h"
#include "TaskCoaxEnvironment.h"

class TaskTakeOff : public TaskDefinition
{
	protected:
		SBApiSimpleContext *interface;
	public:
		/* Env must be a TaskCoaxEnvironment */
		TaskTakeOff(TaskEnvironment *env);
		virtual ~TaskTakeOff() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_TAKEOFF_H
