
#include "task_manager_lib/SequenceTask.h"

using namespace task_manager_lib;
using namespace std;

SequenceTask::SequenceTask(const std::vector<task_manager_msgs::TaskDescriptionLight> &tasks_sequence,TaskScheduler *ts) :
	TaskDefinition("SequenceTask","Run a sequence of tasks",true,-1.0),sequence_id(-1),task_id(0)
{
	sequence=DescriptionLightToTaskParameters(tasks_sequence);
	that=ts;
}

std::vector<std::pair<std::string, TaskParameters > > SequenceTask::DescriptionLightToTaskParameters(const std::vector<task_manager_msgs::TaskDescriptionLight>& tasks)
{
	std::vector<std::pair<std::string, TaskParameters > > output;
	for (unsigned int j=0;j<tasks.size();j++)
	{
		std::pair<std::string,TaskParameters> current_task;
		current_task.first=tasks[j].name;
		TaskParameters current_params;
		current_params.setParameter("task_period",0.5 );

		for (unsigned int i=0;i<tasks[j].parameters.size();i++)
		{	
			if (tasks[j].parameters[i].type=="bool")
			{
				dynamic_reconfigure::BoolParameter bp;
				bp.name=tasks[j].parameters[i].name;
				if (tasks[j].parameters[i].value=="True")
				{
					bp.value=true;
				}
				else
				{
					bp.value=false;
				}
				current_params.bools.push_back(bp);
			}
			else if (tasks[j].parameters[i].type=="double")
			{
				dynamic_reconfigure::DoubleParameter dp;
				dp.name=tasks[j].parameters[i].name;
				dp.value=atof(tasks[j].parameters[i].value.c_str());
				current_params.doubles.push_back(dp);
			}
			else if (tasks[j].parameters[i].type=="int")
			{
				dynamic_reconfigure::IntParameter ip;
				ip.name=tasks[j].parameters[i].name;
				ip.value=atoi(tasks[j].parameters[i].value.c_str());
				current_params.ints.push_back(ip);
			}
			else 
			{
				dynamic_reconfigure::StrParameter sp;
				sp.name=tasks[j].parameters[i].name;
				sp.value=tasks[j].parameters[i].value;
				current_params.strs.push_back(sp);
			}
		}
		current_task.second=current_params;
		output.push_back(current_task);
	}
	return output;
}

TaskIndicator SequenceTask::iterate()
{
    that->keepAliveSequence();
    if (sequence_id==-1)//first thread to launch
    {
    	sequence_id++;
    	task_id=that->launchTask(sequence[sequence_id].first,sequence[sequence_id].second);
    	return TaskStatus::TASK_RUNNING;
    }
    else 
    {
		int current_status=that->getstatus(task_id);
		if (current_status<TaskStatus::TASK_NEWBORN)//task not found
		{
			return TaskStatus::TASK_FAILED;
		}
		else if((current_status>=TaskStatus::TASK_NEWBORN) && !(current_status&TaskStatus::TASK_TERMINATED))
		{
			return TaskStatus::TASK_RUNNING;
		}
		else if(current_status&TaskStatus::TASK_TERMINATED)
		{
            sequence_id++;
			if (sequence_id<(signed)sequence.size())
			{
				task_id=that->launchTask(sequence[sequence_id].first,sequence[sequence_id].second);
				return TaskStatus::TASK_RUNNING;
			}
			else
			{
                task_id = 0;
				return TaskStatus::TASK_COMPLETED;
			}
		}
		else
		{
			that->terminateTask(task_id);
			return current_status;
		}
	}
}

TaskIndicator SequenceTask::terminate()
{
    if (sequence_id < (signed)sequence.size()) {
        that->terminateTask(task_id);
    }
	return TaskStatus::TASK_TERMINATED ;
}


