#include <stdlib.h>
#include <stdio.h>

#include <fstream> 
#include <stdint.h>
#include "task_manager_lib/TaskServerInterface.h"
#include <ros/ros.h>
#include <ros/package.h>

using namespace boost::posix_time;
using namespace std;
using namespace task_manager_lib;
TaskServerInterface::TaskServerInterface(task_manager_lib::TaskScheduler &ts_ref)
{
	saveBasicMissionSrv = ts_ref.n.advertiseService("save_basic_mission", &TaskServerInterface::saveBasicMission,this);
	listMissionSrv=ts_ref.n.advertiseService("list_mission", &TaskServerInterface::listMission,this);
}

//callback

bool TaskServerInterface::saveBasicMission(task_manager_lib::SaveMission::Request  &req, task_manager_lib::SaveMission::Response &res )
{
	cout<<"in save basic mission";
	createBasicMissionFile(req.mission,res.filename);
	return true;
}

bool TaskServerInterface::listMission(task_manager_lib::ListMission::Request  &req, task_manager_lib::ListMission::Response &res )
{
	cout<<"in list mission\n";
	parseMissionDirectory(res.missions);
	return true;
}

void TaskServerInterface::createBasicMissionFile(std::vector<task_manager_msgs::TaskDescriptionLight> &mission, std::string &filename) const
{
	cout<<"here"<<endl;
	std::vector<task_manager_msgs::TaskDescriptionLight> tasklist=mission;
	try
	{
		std::stringstream path(ros::package::getPath("task_manager_turtlesim"));
		cout<<path.str().c_str()<<"\n"<<endl;
		if (path.str().length() >0)
		{
			boost::filesystem::path mission_path(path.str()+"/missions");
			if (boost::filesystem::exists(mission_path))
			{
				time_facet *facet = new time_facet("%d-%b-%Y-%H_%M_%S");
				
				std::stringstream pathName;
				pathName.imbue(locale(pathName.getloc(), facet));
				pathName<<mission_path.string()<<"/mission_"<<second_clock::local_time() << ".mission";
				
				boost::filesystem::path output_boost_path(pathName.str());
				
				if (boost::filesystem::exists(output_boost_path))
				{
					//replace
				}
				else
				{
					ofstream outputFile (pathName.str().c_str()) ;
					//write mission file
					
					//Mission task
					for (unsigned int i = 0;i<mission.size();i++) 
					{ 
						cout<<"in task"<<endl;
						
						outputFile<<mission[i].name;
						outputFile<<"(";
						
						cout<<mission[i].parameters.size();
						
						for (unsigned int j = 0;j<mission[i].parameters.size();j++)
						{
							cout<<"in param"<<endl;
							if (j==0)
							{
								outputFile<<mission[i].parameters[j].name;
								outputFile<<"=";
								outputFile<<mission[i].parameters[j].value;
							}
							else
							{
								outputFile<<",";
								outputFile<<mission[i].parameters[j].name;
								outputFile<<"=";
								outputFile<<mission[i].parameters[j].value;
							}
						}
						outputFile<<")\n"; 
					}
					outputFile<<"\n";
										
					outputFile.close();
					
				}
			filename=output_boost_path.filename().string();
			}
			else
			{
				cout<<"mission path does not exist\n"<<endl;
			}
		
		}
		else
		{
			cout<<"package not found\n"<<endl;
		}
		
		if (mission.size()>0)
		{
		std::cout<<mission[0]<<std::endl;
		}
		
	}
	catch (char * str)
	{
		filename="error creating ";
	}
	
}
	

void TaskServerInterface::parseMissionFile(boost::filesystem::path &mission_file_path, std::vector<task_manager_msgs::MissionList>&output) const
{
	if (boost::filesystem::exists(mission_file_path))
	{
		stringstream pathName(mission_file_path.string());
		ifstream MisssionFile(pathName.str().c_str());
		std::string line;
		task_manager_msgs::MissionList mission_elem;
		std::vector<task_manager_msgs::TaskDescriptionLight> tasks;
		
		if (MisssionFile.is_open())
		{
			std::string taskclient_name("None");
			mission_elem.name=mission_file_path.string();
			while ( getline(MisssionFile,line))
			{
				if (taskclient_name=="None")
				{
					size_t pos_taskclient = line.find("TaskClient(");
					if (pos_taskclient!=string::npos)
					{
						taskclient_name=line.substr (0,pos_taskclient);
						taskclient_name.erase( remove( taskclient_name.begin(), taskclient_name.end(), ' ' ), taskclient_name.end() );
						taskclient_name.erase( remove( taskclient_name.begin(), taskclient_name.end(), '=' ), taskclient_name.end() );
					}
					
				}
				else
				{
					size_t pos = line.find(taskclient_name+".");
					
					if (pos!=string::npos)
					{
						task_manager_msgs::TaskDescriptionLight current_task;
						std::vector<task_manager_msgs::TaskParameter> parameters;
						std::string task=line.substr(pos+taskclient_name.size()+1);
						string task_name(task.substr(0,task.find( "(" )));
						string params=task.substr(task.find( "(" )+1);
						bool one_param=true;
						params=params.substr(0, params.size()-1); //remove ")"
						current_task.name=task_name;
						//check if several params
						do
						{
							task_manager_msgs::TaskParameter parameters_element;
							if (params.find(",")==string::npos)
							{
								if (params.find( "=" )!=string::npos)
								{
									string param=params.substr(0,params.find( "=" ));
									param.erase( remove( param.begin(), param.end(), ' ' ), param.end() ); //remove space
									string value=params.substr(params.find( "=" )+1,params.size()-1);
									value.erase( remove( value.begin(), value.end(), ' ' ), value.end() ); //remove space
									//put in parameter msg
									parameters_element.name=param;
									parameters_element.value=value;
									//put ini paramters array
									parameters.push_back(parameters_element);
								}
							}
							else
							{
								string part1=params.substr(0,params.find( "," ));
								string part2=params.substr(params.find( "," )+1,params.size()-1);
								string param=part1.substr(0,part1.find( "=" ));
								param.erase( remove( param.begin(), param.end(), ' ' ), param.end() ); //remove space
								string value=part1.substr(part1.find( "=" )+1,part1.size()-1);
								value.erase( remove( value.begin(), value.end(), ' ' ), value.end() ); //remove space
								//put in parameter msg
								parameters_element.name=param;
								parameters_element.value=value;
								//put in paramters array
								parameters.push_back(parameters_element);
								params=part2;
								one_param=false;
							}
						}while (params.find(",")!=string::npos); 

						if (!one_param) //in this case first and last element is the same
						{
							task_manager_msgs::TaskParameter parameters_element;
							string param= params.substr (0, params.find( "=" ));
							param.erase( remove( param.begin(), param.end(), ' ' ), param.end() ); //remove space
							string value= params.substr (params.find( "=" )+1,params.size()-1);
							value.erase( remove( value.begin(), value.end(), ' ' ), value.end() ); //remove space
							//put in parameter msg
							parameters_element.name=param;
							parameters_element.value=value;
							//put in paramters array
							parameters.push_back(parameters_element);

						}
						//put in task_manager_msgs/TaskDescriptionLight
						current_task.parameters=parameters;
						//put in std::vector<task_manager_msgs/TaskDescriptionLight>
						tasks.push_back(current_task);
					}
				}
			}
		}
		mission_elem.name=mission_file_path.filename().string();
		mission_elem.mission=tasks;
		output.push_back(mission_elem);
	}
		
			
}

void TaskServerInterface::parseMissionDirectory(std::vector<task_manager_msgs::MissionList>&output) const
{
	try
	{
	//todo for all package
	std::stringstream path(ros::package::getPath("task_manager_turtlesim"));

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
						boost::filesystem::path current_path(boost::filesystem::absolute(iter-> path()));
						parseMissionFile( current_path, output);
					}
				}
				
				for (unsigned int i=0;i<output.size();i++)
				{
					cout <<"-------" <<endl;
					cout <<output[i].name <<endl;
					cout <<"Params" <<endl;
					for (unsigned int j=0;j<output[i].mission.size();j++)
					{
						for (unsigned int k=0;k<output[i].mission[j].parameters.size();k++) 
						{
							cout<< output[i].mission[j].parameters[k].name << " vaut "<<output[i].mission[j].parameters[k].value<<"\n";
						}
					}
				}
				
			}
			else
			{
				cout<<"Can't find missions directory in "<<path.str()<<endl;
			}
		}
	}
	catch(char * str)
	{
		cout<<str<<"\n";
	}
}




