#ifndef TASK_YAW_H
#define TASK_YAW_H

#include "TaskDefinition.h"
#include "TaskCoaxEnvironment.h"

class TaskYaw : public TaskDefinition
{
	protected:
		SBApiSimpleContext *interface;
		double desired_yaw;
		double precision;
		double altinit;
	public:
		/* Env must be a TaskCoaxEnvironment */
		TaskYaw(TaskEnvironment *env);
		virtual ~TaskYaw() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_YAW_H
