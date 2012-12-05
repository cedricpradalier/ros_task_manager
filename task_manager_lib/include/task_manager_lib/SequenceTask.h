#ifndef SEQUENCE_TASK_DEFINITION_H
#define SEQUENCE_TASK_DEFINITION_H

#include "TaskScheduler.h"
using namespace task_manager_msgs;

namespace task_manager_lib {

    class SequenceTask : public TaskDefinition
    {
        protected:
            std::vector<std::pair<std::string,TaskParameters > > sequence;
			TaskScheduler *that;
			int sequence_id;
			unsigned int task_id;

        public:
            // Load a class from a library file fname, and pass env to its
            // constructor. 
            SequenceTask(const std::vector<task_manager_msgs::TaskDescriptionLight> &tasks_sequence, TaskScheduler *ts);
            virtual ~SequenceTask(){};

            //
            // All virtual function below are just forwarding their function to the
            // loaded class.
            //

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

			virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter)
			{
				return TaskStatus::TASK_CONFIGURED;
			};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter)
            {
            	return TaskStatus::TASK_INITIALISED;
            };

            virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh)
            {
            	return TaskParameters();
            };

            virtual TaskParameters getDefaultParameters() const
            {
            	return TaskParameters();
            };

            virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const
            {
            	return dynamic_reconfigure::ConfigDescription();
            };
            
             virtual boost::shared_ptr<TaskDefinition> getInstance() {
                return boost::shared_ptr<TaskDefinition>(new SequenceTask(*this));
            };
            
            
            virtual std::vector<std::pair<std::string, TaskParameters > > DescriptionLightToTaskParameters(const std::vector<task_manager_msgs::TaskDescriptionLight>& tasks);
            //blabalabl
    };


};













#endif // SEQUENCE_TASK_DEFINITION_H
