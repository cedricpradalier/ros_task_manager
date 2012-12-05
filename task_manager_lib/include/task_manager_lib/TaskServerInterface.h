#ifndef TASK_SERVER_INTERFACE_H
#define TASK_SERVER_INTERFACE_H


#include <vector>
#include <set>
#include <map>
#include <ros/ros.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/filesystem.hpp"

#include "TaskScheduler.h"

#include "task_manager_lib/SaveBasicMission.h"
#include "task_manager_lib/SaveComplexMission.h"
#include "task_manager_lib/ListMissions.h"
#include "task_manager_lib/ExeComplexMission.h"
#include "task_manager_lib/StopComplexMission.h"

#include "task_manager_msgs/ComplexMission.h"
#include "task_manager_msgs/BasicMission.h"

#include "task_manager_msgs/TaskMission.h"
#include "task_manager_msgs/TaskDescription.h"

namespace task_manager_lib {

class TaskServerInterface
{
	protected:
		
		//ros service
		ros::ServiceServer saveBasicMissionSrv;
		ros::ServiceServer saveComplexMissionSrv;
		ros::ServiceServer listMissionsSrv;
		ros::ServiceServer executeComplexMissionsSrv;
		ros::ServiceServer stopComplexMissionsSrv;
		//store missions
		std::vector<task_manager_msgs::ComplexMission> ComplexMissions;
		std::vector<task_manager_msgs::BasicMission> BasicMissions;

		//package name
		static std::string package_name;
		
		// ROS callbacks
		bool saveBasicMission(task_manager_lib::SaveBasicMission::Request  &req,task_manager_lib::SaveBasicMission::Response &res );
		bool saveComplexMission(task_manager_lib::SaveComplexMission::Request  &req,task_manager_lib::SaveComplexMission::Response &res );
		bool executeComplexMission(task_manager_lib::ExeComplexMission::Request  &req,task_manager_lib::ExeComplexMission::Response &res);
		bool listMissions(task_manager_lib::ListMissions::Request  &req, task_manager_lib::ListMissions::Response &res );
		bool stopComplexMission(task_manager_lib::StopComplexMission::Request  &req, task_manager_lib::StopComplexMission::Response &res);
		//create python mission file
		void createBasicMissionFile(std::vector<task_manager_msgs::TaskDescriptionLight> &basic_mission,std::string &filename) const;
		void createComplexMissionFile(std::string &complex_mission,std::string &filename);
		
		// parse mission diretory
		void parseMissionDirectory(std::vector<task_manager_msgs::BasicMission>& basic_missions,std::vector<task_manager_msgs::ComplexMission>& complex_missions) ;
		// parse mission files
		void parseBasicMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::BasicMission>& basic_missions) ;
		void parseComplexMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::ComplexMission>& complex_missions) ;
		//execute mission
		void launchComplexMission(std::string &mission_name, int32_t &pid) const;
		//abort mission_
		void abordComplexMission(int &pid);
		void split(const std::string &s, char delim, std::vector<std::string> &elems);


	public:
		// Default constructor:
		TaskServerInterface(task_manager_lib::TaskScheduler &ts_ref);
		
};
};
#endif // TASK_SERVER_INTERFACE_H
