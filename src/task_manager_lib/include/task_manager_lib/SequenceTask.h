#ifndef SEQUENCE_TASK_DEFINITION_H
#define SEQUENCE_TASK_DEFINITION_H
#include <vector>

#include "task_manager_lib/MinimalTaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"
#include "TaskScheduler.h"
using namespace task_manager_msgs;

namespace task_manager_lib {

    class SequenceTask : public TaskInstance<MinimalTaskConfig,TaskEnvironment> 
    {
        protected:
			int sequence_id;
			unsigned int current_task_id;
        public:
            SequenceTask(TaskDefinitionPtr def, TaskEnvironmentPtr ev) :
                Parent(def,ev),sequence_id(-1),current_task_id(0) {}
            virtual ~SequenceTask() {} 

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };


    class SequenceDef : public TaskDefinition<MinimalTaskConfig,TaskEnvironment,SequenceTask>
    {
        protected:
            std::vector<std::pair<std::string,TaskParameters > > sequence;
			TaskScheduler *sched;

            friend class SequenceTask;

        public:
            SequenceDef(const std::vector<task_manager_msgs::TaskDescriptionLight> &tasks_sequence, 
                    TaskScheduler *ts);
            virtual ~SequenceDef(){};

            virtual std::vector<std::pair<std::string, TaskParameters > > DescriptionLightToTaskParameters(const std::vector<task_manager_msgs::TaskDescriptionLight>& tasks);
    };




};













#endif // SEQUENCE_TASK_DEFINITION_H
