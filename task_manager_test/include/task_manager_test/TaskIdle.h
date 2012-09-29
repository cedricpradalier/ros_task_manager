#ifndef TASK_IDLE_H
#define TASK_IDLE_H

#include "task_manager_lib/TaskDefinition.h"

class TaskIdle : public TaskDefinition
{
	protected:
	public:
		TaskIdle(TaskEnvironment *env) : TaskDefinition("Idle","Do nothing",true,-1) {}
		virtual ~TaskIdle() {};

		virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

		virtual TaskIndicator iterate();

		virtual TaskIndicator terminate();

        virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh);

        virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const {
            return dynamic_reconfigure::ConfigDescription();
        }
};

#endif // TASK_IDLE_H
