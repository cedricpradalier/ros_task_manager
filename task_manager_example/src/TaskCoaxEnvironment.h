#ifndef TASK_COAX_ENVIRONMENT_H
#define TASK_COAX_ENVIRONMENT_H

#include "TaskDefinition.h"

#include <com/sbsimple.h>

class TaskCoaxEnvironment : public TaskEnvironment
{
	public:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
	public:
		TaskCoaxEnvironment(SBApiSimpleContext *s) : 
			simple(s), sensorList(0) {}
		~TaskCoaxEnvironment() {}


};


#endif // TASK_COAX_ENVIRONMENT_H
