#ifndef TASK_SERVO_H
#define TASK_SERVO_H

#include "TaskDefinition.h"
#include "TaskCoaxEnvironment.h"
#include "PID.h"

class TaskServo : public TaskDefinition
{
	protected:
		SBApiSimpleContext *interface;
		double desired_distance;
		double precision;
		int sensor;
		double yawinit,altinit;
		PID pid;
	public:
		/* Env must be a TaskCoaxEnvironment */
		TaskServo(TaskEnvironment *env);
		virtual ~TaskServo() {};

		virtual TaskStatus configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskStatus iterate();

		virtual TaskStatus terminate();

};

#endif // TASK_SERVO_H
