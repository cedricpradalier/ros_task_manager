#ifndef TASK_TEST_H
#define TASK_TEST_H

#include "task_manager_lib/TaskDefinition.h"

class TaskTest : public TaskDefinition
{
	protected:
		double duration;
		unsigned int counter;
	public:
		TaskTest(TaskEnvironment *env) : TaskDefinition("Test","Do nothing",true,5.0), counter(0) {}
		virtual ~TaskTest() {};

		virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator iterate();

		virtual TaskIndicator terminate();

        virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh);

        virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const ;
};

#endif // TASK_TEST_H
