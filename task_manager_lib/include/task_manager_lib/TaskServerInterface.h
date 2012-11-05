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

#include "task_manager_lib/SaveBasicMission.h"
#include "task_manager_lib/SaveComplexMission.h"
#include "task_manager_lib/ListMissions.h"


#include "task_manager_msgs/ComplexMission.h"
#include "task_manager_msgs/BasicMission.h"

#include "task_manager_msgs/TaskMission.h"
#include "task_manager_msgs/TaskDescription.h"

namespace task_manager_lib {

class TaskServerInterface
{
	protected:
		
		ros::ServiceServer saveBasicMissionSrv;
		ros::ServiceServer saveComplexMissionSrv;
		ros::ServiceServer listMissionsSrv;

		
		// ROS callbacks
		bool saveBasicMission(task_manager_lib::SaveBasicMission::Request  &req,task_manager_lib::SaveBasicMission::Response &res );
		bool saveComplexMission(task_manager_lib::SaveComplexMission::Request  &req,task_manager_lib::SaveComplexMission::Response &res );
		
		bool listMissions(task_manager_lib::ListMissions::Request  &req, task_manager_lib::ListMissions::Response &res );

		//create a python mission file
		void createBasicMissionFile(std::vector<task_manager_msgs::TaskDescriptionLight> &basic_mission,std::string &filename) const;
		void createComplexMissionFile(std::string &complex_mission,std::string &filename) const;
		
		// parse mission diretory
		void parseMissionDirectory(std::vector<task_manager_msgs::BasicMission>& basic_missions,std::vector<task_manager_msgs::ComplexMission>& complex_missions) const;
		// parse mission files
		void parseBasicMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::BasicMission>& basic_missions) const;
		void parseComplexMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::ComplexMission>& complex_missions) const;


	public:
		// Default constructor:
		TaskServerInterface(task_manager_lib::TaskScheduler &ts_ref);
		
};
};
#endif // TASK_SERVER_INTERFACE_H
