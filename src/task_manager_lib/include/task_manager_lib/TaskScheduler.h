#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <vector>
#include <set>
#include <map>

#include "TaskDefinition.h"
#include "TaskHistory.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "task_manager_lib/StartTask.h"
#include "task_manager_lib/StopTask.h"
#include "task_manager_lib/GetTaskList.h"
#include "task_manager_lib/GetTaskListLight.h"
#include "task_manager_lib/GetAllTaskStatus.h"
#include "task_manager_lib/GetHistory.h"
#include "task_manager_lib/ExeTaskSequence.h"


#include "task_manager_msgs/TaskStatus.h"
#include "task_manager_msgs/TaskDescriptionLight.h"
#include "task_manager_msgs/TaskHistory.h"

#include <iostream>

#include <boost/thread.hpp>  
#include <boost/thread/mutex.hpp>  
#include <boost/thread/condition_variable.hpp>  

namespace task_manager_lib {
#define TASK_STATUS_MASK 0xFFl
#define TASK_FOREGROUND 0x100l
#define TASK_BACKGROUND 0x000l
#define PRINTF(level,X...) if (level <= (signed)TaskScheduler::debug) ROS_INFO(X)
	// Class that manages the execution of task directly or through ROS services
	class TaskScheduler
	{
		public:
			typedef unsigned int TaskId;
			static unsigned int debug;
		protected:
		
			struct ThreadParameters {
				static unsigned int gtpid;

				bool foreground,running;
				unsigned int tpid;
				boost::shared_ptr<TaskInstanceBase> task;
				TaskParameters params;
				TaskScheduler *that;
				double period;
                boost::shared_ptr<boost::thread> tid;
                boost::mutex task_mutex;
                boost::condition_variable task_condition;
                boost::mutex aperiodic_task_mutex;
                boost::condition_variable aperiodic_task_condition;

				ros::Time statusTime;
				TaskIndicator status;
				std::string statusString;
				ros::Publisher statusPub;
				
				ThreadParameters(ros::Publisher pub, TaskScheduler *ts, 
						boost::shared_ptr<TaskDefinitionBase> td, 
						double tperiod);
				ThreadParameters(const ThreadParameters & tp);
				~ThreadParameters();

				task_manager_msgs::TaskStatus getRosStatus() {
					task_manager_msgs::TaskStatus st = task->getRosStatus();
					st.id = tpid;
					st.status = status | 
						(foreground?TASK_FOREGROUND:TASK_BACKGROUND);
					st.status_time = statusTime;
                    st.plist = params;
					return st;
				}
				
				
                bool isAnInstanceOf(const boost::shared_ptr<TaskDefinitionBase> & def) {
                    return task->isAnInstanceOf(def);
                }


				void setStatus(TaskIndicator newstatus, const std::string & text, const ros::Time & tnow) {
                    task->setStatus(newstatus);
                    task->setStatusString(text);
                    updateStatus(tnow);
				}

				void updateStatus(const ros::Time & tnow) {
					status = task->getStatus();
					statusString = task->getStatusString();
					statusTime = tnow;
					statusPub.publish(getRosStatus());
					manageHistory();
					//ROS_INFO("Pub: task %d %s %s",tpid, task->getName().c_str(),taskStatusToString(status));
				}

				
				void manageHistory()
				{
					if (status==task_manager_msgs::TaskStatus::TASK_INITIALISED)
					{
						std::vector<TaskHistory>::iterator it_vector;
						bool task_exist=false;
						for ( it_vector=that->history.begin() ; it_vector < that->history.end(); it_vector++ )
						{
							if (it_vector->getid()==tpid)
							{
								task_exist=true;
								break;
							}
						}
						if (task_exist==false)
						{
							TaskHistory current_task(tpid,task->getName().c_str(),params,statusTime,status) ;
							if (that->history.size()< history_size)
							{
								that->history.push_back(current_task);
							}
							else
							{
								that->history.erase(that->history.begin());
								that->history.push_back(current_task);
							}
						}
					}
					
					if (status>=task_manager_msgs::TaskStatus::TASK_RUNNING )
					{
						std::vector<TaskHistory>::iterator it_vector;
						for ( it_vector=that->history.begin() ; it_vector < that->history.end(); it_vector++ )
						{
							if (it_vector->getid()==tpid)
							{
								it_vector->updateTaskHistory(statusTime,status);
								break;
							}
						}
					}
					
				}
				
				bool operator<(const ThreadParameters & tp) {
					return tpid < tp.tpid;
				}
			};
			
			
			
		protected:
			ros::NodeHandle n;
			ros::Time lastKeepAlive;
			ros::Subscriber keepAliveSub;
			ros::Publisher statusPub;
			ros::ServiceServer startTaskSrv;
			ros::ServiceServer stopTaskSrv;
			ros::ServiceServer getTaskListSrv;
			ros::ServiceServer getAllTaskStatusSrv;
			ros::ServiceServer getTaskListLightSrv;
			ros::ServiceServer getHistorySrv;
			ros::ServiceServer executeSequenceTasksSrv;
			void keepAliveCallback(const std_msgs::Header::ConstPtr& msg);

		protected:
			boost::shared_ptr<TaskDefinitionBase> idle;
			typedef std::map<std::string,boost::shared_ptr<TaskDefinitionBase>,std::less<std::string> > TaskDirectory;
			typedef std::pair<unsigned int, boost::shared_ptr<ThreadParameters> > TaskSetItem;
			typedef std::map<unsigned int, boost::shared_ptr<ThreadParameters>, std::less<unsigned int> > TaskSet;
			std::vector<TaskHistory> history;
			TaskDirectory tasks;
			TaskSet runningThreads,zombieThreads;
			static void printTaskSet(const std::string & name, const TaskSet & ts);
			boost::shared_ptr<ThreadParameters> mainThread;

		protected:
			static const double IDLE_TIMEOUT;
			static const double DELETE_TIMEOUT;
			static const unsigned int history_size; 
			// Action management: create task, terminate task
			typedef enum {
                NO_ACTION,
				START_IDLE_TASK, 
				START_TASK,
				CONDITIONALLY_IDLE,
				DELETE_TASK,
				WAIT_CANCELLED
			} ActionType;
			struct ThreadAction {
				ActionType type;
				boost::shared_ptr<ThreadParameters> tp;
                ThreadAction() : type(NO_ACTION) {}
			};
			static const char * actionString(ActionType at);

			// Action queueing mecanism. Action are used to manage the threads
			// associated with each task.
			typedef std::map<double, ThreadAction, std::less<double> > ActionQueue;
			ActionQueue actionQueue;
			ThreadAction getNextAction();
			void enqueueAction(ActionType type,boost::shared_ptr<ThreadParameters> tp);
			void enqueueAction(const ros::Time & when, ActionType type,boost::shared_ptr<ThreadParameters> tp);
			void removeConditionalIdle();
			bool runScheduler;
            boost::thread aqid;
            boost::mutex aqMutex;
            boost::condition_variable aqCond;
			int runSchedulerLoop();


		protected:
			double idleTimeout;
			double startingTime;
			double defaultPeriod;
            boost::mutex scheduler_mutex;
            boost::condition_variable scheduler_condition;

			TaskId launchTask(boost::shared_ptr<ThreadParameters> tp);
			void runTask(boost::shared_ptr<ThreadParameters> tp);
			void runAperiodicTask(boost::shared_ptr<ThreadParameters> tp);
			void terminateTask(boost::shared_ptr<ThreadParameters> tp);
			void cleanupTask(boost::shared_ptr<ThreadParameters> tp);
			void deleteTask(boost::shared_ptr<ThreadParameters> tp);

			// All ROS callbacks
			bool startTask(task_manager_lib::StartTask::Request  &req,
					task_manager_lib::StartTask::Response &res );
			bool stopTask(task_manager_lib::StopTask::Request  &req,
					task_manager_lib::StopTask::Response &res );
			bool getAllTaskStatus(task_manager_lib::GetAllTaskStatus::Request  &req,
					task_manager_lib::GetAllTaskStatus::Response &res );
			bool getTaskList(task_manager_lib::GetTaskList::Request  &req,
					task_manager_lib::GetTaskList::Response &res );
			bool getTaskListLight(task_manager_lib::GetTaskListLight::Request  &req, task_manager_lib::GetTaskListLight::Response &res );
			bool getHistory(task_manager_lib::GetHistory::Request  &req, task_manager_lib::GetHistory::Response &res );
			bool executeTaskSequence(task_manager_lib::ExeTaskSequence::Request  &req,task_manager_lib::ExeTaskSequence::Response &res);

			// Generate a task description list, ready to be published by ROS
			void generateTaskList(std::vector<task_manager_msgs::TaskDescription> & tlist) const;
			// Generate a task status vector, ready to be published by ROS
			void generateTaskStatus(std::vector<task_manager_msgs::TaskStatus> & running,
					std::vector<task_manager_msgs::TaskStatus> & zombies) ;
			void generateTaskListLight(std::vector<task_manager_msgs::TaskDescription> &input,std::vector<task_manager_msgs::TaskDescriptionLight> &output) const;
			void generateHistory(std::vector<task_manager_msgs::TaskHistory> &output) ;
			void launchTaskSequence(std::vector<task_manager_msgs::TaskDescriptionLight> &tasks, int & id);

			// convenience function
			ros::Time now() { return ros::Time::now();}
			
			void DescriptionLightToTaskParameters(const task_manager_msgs::TaskDescriptionLight& task_params, TaskParameters& output);
		public:
			// Default constructor:
			// nh: ros NodeHandle in which all the services and topic will  be
			// created.
			// idle: a pointer to the definition of the idle class
			// deftPeriod: the default period for task execution
			TaskScheduler(ros::NodeHandle & nh, boost::shared_ptr<TaskDefinitionBase> idle, double deftPeriod);
			~TaskScheduler();

			// Cleanup all the task
			int terminateAllTasks();
			// Print the task directory (added task plus dynamic tasks)
			void printTaskDirectory(bool with_ros=true) const;
			// Return the task directory
			const TaskDirectory & getDirectory() const {return tasks;}

			// Call the configure function of all the tasks
			void configureTasks();

			// Start the scheduling thread
			int startScheduler();
			// Stop the scheduling thread
			int stopScheduler();


			// Start a task by name and parameters
			TaskId launchTask(const std::string & taskname, const TaskParameters & tp);
			// Start the idle task, eventually terminating the current foreground
			TaskId launchIdleTask();
			// Wait for a given task to complete
			int waitTaskCompletion(TaskId id, double timeout);


			// Add a task to the directory
			void addTask(boost::shared_ptr<TaskDefinitionBase> task);

			// Load a task from a file to the directory, and create it with
			// argument env
			void loadTask(const std::string & filename, boost::shared_ptr<TaskEnvironment> env);
			// Load all task from a folder and initialise all of them with env.
			void loadAllTasks(const std::string & dirname, boost::shared_ptr<TaskEnvironment> env);

            // Remove all the dynamic tasks from the list of known tasks
            void clearAllDynamicTasks();
			
			double getdfltPeriod(){return defaultPeriod;};
			
			int getstatus(unsigned int &taskid);
			
			int terminateTask(unsigned int &taskid);
			
			void keepAliveSequence();
			
			ros::NodeHandle getNodeHandle();
			
			
	};


};


#endif // TASK_SCHEDULER_H
