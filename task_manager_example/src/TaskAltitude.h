#ifndef TASK_ALTITUDE_H
#define TASK_ALTITUDE_H

#include "TaskDefinition.h"
#include "TaskCoaxEnvironment.h"

class TaskAltitude : public TaskDefinition
{
	protected:
		SBApiSimpleContext *interface;
		double desired_altitude;
		double precision;
		double yawinit;
	public:
		/* Env must be a TaskCoaxEnvironment */
		TaskAltitude(TaskEnvironment *env);
		virtual ~TaskAltitude() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_ALTITUDE_H
