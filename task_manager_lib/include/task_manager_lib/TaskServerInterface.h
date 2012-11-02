#ifndef TASK_SERVER_INTERFACE_H
#define TASK_SERVER_INTERFACE_H


#include <vector>
#include <set>
#include <map>
#include <ros/ros.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/filesystem.hpp"

#include "TaskScheduler.h"

#include "task_manager_lib/SaveMission.h"
#include "task_manager_lib/ListMission.h"

#include "task_manager_msgs/TaskMission.h"
#include "task_manager_msgs/TaskDescription.h"

namespace task_manager_lib {

class TaskServerInterface
{
	protected:
		
		ros::ServiceServer saveBasicMissionSrv;
		ros::ServiceServer listMissionSrv;
		
		// ROS callbacks
		bool saveBasicMission(task_manager_lib::SaveMission::Request  &req,task_manager_lib::SaveMission::Response &res );
		bool listMission(task_manager_lib::ListMission::Request  &req, task_manager_lib::ListMission::Response &res );

		//create a python mission file
		void createBasicMissionFile(std::vector<task_manager_msgs::TaskDescriptionLight> &mission,std::string &filename) const;
		// parse mission diretory
		void parseMissionDirectory(std::vector<task_manager_msgs::MissionList>&output) const;
		void parseMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::MissionList>&output) const;

	public:
		// Default constructor:
		TaskServerInterface(task_manager_lib::TaskScheduler &ts_ref);
		
};
};
#endif // TASK_SERVER_INTERFACE_H
