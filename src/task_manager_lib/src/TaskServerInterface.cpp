#include <stdlib.h>
#include <stdio.h>
#include <boost/version.hpp>

#include <fstream> 
#include <stdint.h>
#include "task_manager_lib/TaskServerInterface.h"
#include <ros/ros.h>
#include <ros/package.h>

using namespace boost::posix_time;
using namespace std;
using namespace task_manager_lib;


std::string TaskServerInterface::package_name = "task_manager_turtlesim";


TaskServerInterface::TaskServerInterface(task_manager_lib::TaskScheduler &ts_ref)
{
	saveBasicMissionSrv = ts_ref.getNodeHandle().advertiseService("save_basic_mission", &TaskServerInterface::saveBasicMission,this);
	saveComplexMissionSrv = ts_ref.getNodeHandle().advertiseService("save_complex_mission", &TaskServerInterface::saveComplexMission,this);
	executeComplexMissionsSrv= ts_ref.getNodeHandle().advertiseService("execute_complex_mission", &TaskServerInterface::executeComplexMission,this);
	stopComplexMissionsSrv= ts_ref.getNodeHandle().advertiseService("abord_complex_mission", &TaskServerInterface::stopComplexMission,this);
	listMissionsSrv=ts_ref.getNodeHandle().advertiseService("list_mission", &TaskServerInterface::listMissions,this);
}

//callback

bool TaskServerInterface::saveBasicMission(task_manager_lib::SaveBasicMission::Request  &req, task_manager_lib::SaveBasicMission::Response &res )
{
	createBasicMissionFile(req.basic_mission,res.filename);
	return true;
}

bool TaskServerInterface::saveComplexMission(task_manager_lib::SaveComplexMission::Request  &req, task_manager_lib::SaveComplexMission::Response &res )
{
	createComplexMissionFile(req.complex_mission,res.filename);
	return true;
}

bool TaskServerInterface::executeComplexMission(task_manager_lib::ExeComplexMission::Request  &req, task_manager_lib::ExeComplexMission::Response &res )
{
	launchComplexMission(req.mission_name, res.pid);
	return true;
}

bool TaskServerInterface::stopComplexMission(task_manager_lib::StopComplexMission::Request  &req, task_manager_lib::StopComplexMission::Response &res )
{
	abordComplexMission(req.pid);
	return true;
}


bool TaskServerInterface::listMissions(task_manager_lib::ListMissions::Request  &req, task_manager_lib::ListMissions::Response &res )
{
	parseMissionDirectory(res.basic_missions,res.complex_missions);
	return true;
}

void TaskServerInterface::createBasicMissionFile(std::vector<task_manager_msgs::TaskDescriptionLight> &basic_mission, std::string &filename) const
{
	std::vector<task_manager_msgs::TaskDescriptionLight> tasklist=basic_mission;
	
	try
	{
		stringstream path(ros::package::getPath(TaskServerInterface::package_name));
		stringstream msg;
		if (path.str().length() >0)
		{
			boost::filesystem::path mission_path(path.str()+"/missions");
			if (boost::filesystem::exists(mission_path))
			{
				time_facet *facet = new time_facet("%d-%b-%Y-%H_%M_%S");
				
				stringstream pathName;
				pathName.imbue(locale(pathName.getloc(), facet));
				pathName<<mission_path.string()<<"/mission_"<<second_clock::local_time() << ".mission";
				
				boost::filesystem::path output_boost_path(pathName.str());
				

				ofstream outputFile (pathName.str().c_str()) ;
				//write mission file
				//Mission task
				for (unsigned int i = 0;i<tasklist.size();i++) 
				{
					outputFile<<"---"<<"\n";
					outputFile<<tasklist[i].name<<"\n";

					if (tasklist[i].parameters.size()==0)
					{
						outputFile<<"empty"<<"\n";
					}
					else
					{
						for (unsigned int j = 0;j<tasklist[i].parameters.size();j++)
						{
							outputFile<<tasklist[i].parameters[j].name;
							outputFile<<";";
							outputFile<<tasklist[i].parameters[j].value;
							outputFile<<"|";
						}
						outputFile<<"\n";
					}
					
					outputFile.close();
					
				}
#if (BOOST_VERSION > 104200)
                filename=output_boost_path.filename().string();
#else
                filename=output_boost_path.filename();
#endif
			}
			else
			{
				#if (BOOST_VERSION > 104200)
					PRINTF(1,"%s does not exist\n",mission_path.c_str());
				#else
					PRINTF(1,"%s does not exist\n",mission_path.string().c_str());
				#endif
			}
		
		}
		else
		{
			PRINTF(1,"%s does not exist\n",path.str().c_str());
		}
		
	}
	catch (char * str)
	{
		PRINTF(1,"%s",str);
		filename="Error creating Basic Mission file";
	}
}

void TaskServerInterface::createComplexMissionFile(std::string &complex_mission, std::string &filename) 
{
	std::string tasklist=complex_mission;
	stringstream msg;
	try
	{
		std::stringstream path(ros::package::getPath(TaskServerInterface::package_name));
		if (path.str().length() >0)
		{
			boost::filesystem::path mission_path(path.str()+"/missions");
			if (boost::filesystem::exists(mission_path))
			{
				time_facet *facet = new time_facet("%d-%b-%Y-%H_%M_%S");
				
				std::stringstream pathName;
				pathName.imbue(locale(pathName.getloc(), facet));
				pathName<<mission_path.string()<<"/mission_"<<second_clock::local_time() << ".py";
				
				boost::filesystem::path output_boost_path(pathName.str());
				
				if (boost::filesystem::exists(output_boost_path))
				{
					//replace
				}
				else
				{
					//write mission file
					try
					{
						ofstream outputFile (pathName.str().c_str()) ;
						outputFile<<tasklist<<endl;
						outputFile.close();
						std::stringstream command_line;
						command_line<<"chmod 775 "<<pathName.str();
						int result=system(command_line.str().c_str());
						if (result!=0)
						{
							PRINTF(1,"Error %d setting execution permission to %s",result,command_line.str().c_str());
						}
					}
					catch(char * str)
					{
						PRINTF(1,"%s",str);
					}
				}
#if BOOST_VERSION > 104200
				filename=output_boost_path.filename().string();
#else
				filename=output_boost_path.filename();
#endif
				//update list of complex filename
				task_manager_msgs::ComplexMission current_mission;
				current_mission.name=filename;
				current_mission.complex_mission=tasklist;
				ComplexMissions.push_back(current_mission);
				
			}
			else
			{
				#if (BOOST_VERSION > 104200)
					PRINTF(1,"%s does not exist\n",mission_path.c_str());
				#else
					PRINTF(1,"%s does not exist\n",mission_path.string().c_str());
				#endif
			}
		}
		else
		{
			PRINTF(1," %s does not exist\n",path.str().c_str());
		}
		
	}
	catch (char * str)
	{
		PRINTF(1,"%s",str);
		filename="error creating file";
	}
	
}


void TaskServerInterface::parseBasicMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::BasicMission>& basic_missions) 
{
	if (boost::filesystem::exists(mission_file_path))
	{
		stringstream pathName(mission_file_path.string());
		ifstream MisssionFile(pathName.str().c_str());
		std::string line;
		
		task_manager_msgs::BasicMission mission_elem;
		std::vector<task_manager_msgs::TaskDescriptionLight> tasks;
		
		if (MisssionFile.is_open())
		{
			task_manager_msgs::TaskDescriptionLight current_task;
			std::vector<task_manager_msgs::TaskParameter> parameters;
			
			unsigned int current_line(0), task_line(0);
			while ( getline(MisssionFile,line))
			{
				current_line ++;
				if (line=="---")
				{
					task_line=current_line;
					current_task=task_manager_msgs::TaskDescriptionLight();
					parameters.clear();
					
				}
				else
				{
					if (current_line>0)
					{
						if (current_line==task_line+1)//name
						{
							current_task.name=line;
						}
						else if (current_line==task_line+2)//params
						{
							if (line!="empty")
							{
								std::vector<std::string> params;
								split(line,'|',params);
								for (unsigned int j=0;j<params.size();j++)
								{
									std::vector<std::string> name_value;
									split(params[j],';',name_value);
									task_manager_msgs::TaskParameter current_param;
									current_param.name=name_value[0];
									current_param.value=name_value[1];
									parameters.push_back(current_param);
								}
								current_task.parameters=parameters;
								tasks.push_back(current_task);
								
							}
						}
						else
						{
							//error
						}
					}
				}
				
			}
		}
#if BOOST_VERSION > 104200
		mission_elem.name=mission_file_path.filename().string();
#else
		mission_elem.name=mission_file_path.filename();
#endif
		mission_elem.basic_mission=tasks;
		//store in taskserverinterface BasicMissions
		BasicMissions.push_back(mission_elem);
		//response
		basic_missions.push_back(mission_elem);
	}
}

void TaskServerInterface::parseComplexMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::ComplexMission>& complex_missions) 
{
	if (boost::filesystem::exists(mission_file_path))
	{
		stringstream pathName(mission_file_path.string());
		ifstream MisssionFile(pathName.str().c_str());
		std::string line;
		task_manager_msgs::ComplexMission mission_elem;
		stringstream current_complex_mission;
		
		if (MisssionFile.is_open())
		{
			while ( getline(MisssionFile,line))
			{
				current_complex_mission<<line<<endl;
			}
		}
#if BOOST_VERSION > 104200
		mission_elem.name=mission_file_path.filename().string();
#else
		mission_elem.name=mission_file_path.filename();
#endif
		mission_elem.complex_mission= current_complex_mission.str();
		//store
		ComplexMissions.push_back(mission_elem);
		//in response
		complex_missions.push_back(mission_elem);
		
	}
}

void TaskServerInterface::parseMissionDirectory(std::vector<task_manager_msgs::BasicMission>& basic_missions,std::vector<task_manager_msgs::ComplexMission>& complex_missions ) 
{
	try
	{
	//todo for all package
	std::stringstream path(ros::package::getPath(TaskServerInterface::package_name));

		if (path.str().length() >0)
		{
			boost::filesystem::path mission_path(path.str()+"/missions");
			if (boost::filesystem::exists(mission_path))
			{
				boost::filesystem::directory_iterator end;
				
				for( boost::filesystem::directory_iterator iter(mission_path) ; iter != end ;iter++ ) 
				{
					if ( !boost::filesystem::is_directory( *iter ) )
					{
#if BOOST_VERSION > 104200
						boost::filesystem::path current_path(boost::filesystem::absolute(iter-> path()));
#else
							//char * cwd = get_current_dir_name();
							//boost::filesystem::path current_path(std::string(cwd) + iter-> path().string());
							//free(cwd);
							boost::filesystem::path current_path(iter-> path().string());
#endif
						
						if (current_path.extension() ==".py")
						{
							parseComplexMissionFile(current_path,complex_missions);
						}
						else if (current_path.extension() ==".mission")
						{
							parseBasicMissionFile(current_path,basic_missions);
						}
					}
				}
				
			}
			else
			{
				PRINTF(1,"Can't find missions directory in %s \n",path.str().c_str());
			}
		}
	}
	catch(char * str)
	{
		PRINTF(1,"%s",str);
	}
}

void TaskServerInterface::launchComplexMission(std::string & mission_name, int &pid) const
{
	
	int id(-1);
	for (unsigned int i=0;i<ComplexMissions.size();i++)
	{
		if (ComplexMissions[i].name==mission_name)
		{
			id=i;
		}
	}
	
	std::string full_name=saveBasicMissionSrv.getService();
	size_t pos=full_name.find("save_basic_mission");
	std::string node_name=full_name.substr(0,pos-1);
	
	stringstream parameter;
	parameter<<"_server:="<<node_name;
	
	stringstream command_line;
	command_line<<"rosrun "<<package_name<<" "<<mission_name<<" "<<parameter.str();
	
	//WARNING adding fork in service 
	stringstream msg;
	if (id>-1)
	{
		int current_pid = fork();

		if (current_pid ==-1)
		{
			PRINTF(1,"Fork failed");
		}
		else if (current_pid ==0)//in child
		{
			
			execl ("/bin/bash","bash","-i","-c",command_line.str().c_str(),(char *) 0);
			//msg<<"Error running the following command :"<<"rosrun "<<package_name.c_str()<<" "<<mission_name.c_str()<<" "<<parameter.str().c_str()<<"\n";
			PRINTF(1,"Error running the following command : 'bash -i -c  %s ' \n",command_line.str().c_str());
		}
		else //in parent
		{
			pid=current_pid;
		}
	}
	else
	{
		PRINTF(1,"Complex Mission %s not found\n",mission_name.c_str());
	}
}

void TaskServerInterface::abordComplexMission(int &pid)
{
	kill( pid, SIGKILL );
}



void TaskServerInterface::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


