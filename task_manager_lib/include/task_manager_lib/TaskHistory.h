#ifndef TASK_HISTORY_H
#define TASK_HISTORY_H

#include "TaskDefinition.h"
namespace task_manager_lib {

	class TaskHistory
	{
		protected:
			unsigned int thid;
			std::string thname;
			ros::Time startTime;
			ros::Time endTime;
			TaskParameters params;
			unsigned int status;
		
		public:
	
			TaskHistory(unsigned int id, const std::string & name, TaskParameters params, const ros::Time & tnow, const unsigned int & statusnb);
			void updateTaskHistory(ros::Time tnow, const unsigned int & statusnb );
			//retrieve data
			unsigned int getid();
			std::string getname();
			const ros::Time getstartTime();
			const ros::Time getendTime();
			TaskParameters getparams();
			unsigned int getstatus();
			//set values
			void setid(unsigned int & current_id);
			void setname(std::string& current_name);
			void setstartTime(ros::Time current_starttime);
			void setendTime( ros::Time current_endtime);
			void setparams(TaskParameters& current_params);
			void setstatus( const unsigned int & current_status);
	};
};
#endif // TASK_HISTORY_H
