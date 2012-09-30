#ifndef TASK_CLIENT_H
#define TASK_CLIENT_H

#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <dynamic_reconfigure/Config.h>
#include "task_manager_lib/TaskScheduler.h"

class TaskClient {
	public:

		struct TaskState {
			TaskScheduler::TaskId id;
			std::string name;
			TaskIndicator status;
			std::string statusString;
            ros::Time statusTime;
			bool foreground;
		};

		typedef std::map<TaskScheduler::TaskId,
				TaskState, std::less<TaskScheduler::TaskId> > StatusMap;
	protected:
        mutable pthread_mutex_t mutex;
		unsigned int messageid;
		std::vector<task_manager_msgs::TaskDescription> taskList; 
		StatusMap taskStatus;

        ros::AsyncSpinner spinner;
        ros::ServiceClient startTaskClt;
        ros::ServiceClient stopTaskClt;
        ros::ServiceClient getTaskListClt;
        ros::ServiceClient getAllTaskStatusClt;

        ros::Subscriber statusSub;
        void statusCallback(const task_manager_msgs::TaskStatus::ConstPtr& msg);
	public:
		TaskClient(const std::string & node, ros::NodeHandle & nh);
		~TaskClient();

		int updateTaskList();
		const std::vector<task_manager_msgs::TaskDescription> & getTaskList() const;
		void printTaskList() const;

		TaskScheduler::TaskId startTask(const std::string & taskname, 
				const TaskParameters & tp);
		TaskScheduler::TaskId startTask(const std::string & taskname, 
				bool foreground, double period,
				const TaskParameters & tp);

		bool startTaskAndWait(const std::string & taskname, 
				const TaskParameters & tp);
		int idle();

		bool waitTask(TaskScheduler::TaskId tid);

		const StatusMap & getStatusMap() const {
            return taskStatus;
        }
		void updateAllStatus();
		void printStatusMap() const;
};


#endif // TASK_CLIENT_H
