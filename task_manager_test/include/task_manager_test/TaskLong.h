#ifndef TASK_LONG_H
#define TASK_LONG_H

#include "task_manager_lib/TaskDefinition.h"

class TaskLong : public TaskDefinition
{
	protected:
		double duration;
		unsigned int counter;
	public:
		TaskLong(TaskEnvironment *env) : TaskDefinition("Long","Wait longly",false,25.0), counter(0) {}
		virtual ~TaskLong() {};

		virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator iterate();

		virtual TaskIndicator terminate();

        virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh);
};

#endif // TASK_LONG_H
