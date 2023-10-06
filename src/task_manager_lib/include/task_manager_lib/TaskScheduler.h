#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <vector>
#include <set>
#include <map>

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_lib/TaskDefinition.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "task_manager_msgs/srv/start_task.hpp"
#include "task_manager_msgs/srv/stop_task.hpp"
#include "task_manager_msgs/srv/get_task_list.hpp"
#include "task_manager_msgs/srv/get_all_task_status.hpp"
#include "task_manager_msgs/msg/task_status.hpp"
#include "task_manager_msgs/msg/task_status.hpp"

#if 0
// Not implemented for ROS2
#include "task_manager_lib/TaskHistory.h"
#include "task_manager_lib/GetTaskListLight.h"
#include "task_manager_lib/GetHistory.h"
#include "task_manager_lib/ExeTaskSequence.h"

#include "task_manager_msgs/TaskDescriptionLight.h"
#include "task_manager_msgs/TaskHistory.h"
#include "task_manager_msgs/EncapsulatedMessage.h"
#endif

#include <iostream>

#include <boost/thread.hpp>  
#include <memory>  
#include <mutex>  
#include <condition_variable>  

namespace task_manager_lib {
#define TASK_STATUS_MASK 0xFFl
#define TASK_FOREGROUND 0x100l
#define TASK_BACKGROUND 0x000l
// #define PRINTF(level,X...) if (level <= (signed)TaskScheduler::debug) ROS_INFO(X)
	// Class that manages the execution of task directly or through ROS services
	class TaskScheduler
	{
		public:
			typedef unsigned int TaskId;
			static unsigned int debug;
		protected:
            typedef std::vector<rcl_interfaces::msg::Parameter> TaskParameters;
		
			struct ThreadParameters {
				static unsigned int gtpid;

				bool foreground,running;
				unsigned int tpid;
				std::shared_ptr<TaskInstanceBase> task;
                TaskConfig params;
				TaskScheduler *that;
				double period;
                std::shared_ptr<boost::thread> tid;
                std::mutex task_mutex;
                std::condition_variable task_condition;
                std::mutex aperiodic_task_mutex;
                std::condition_variable aperiodic_task_condition;

				rclcpp::Time statusTime;
				TaskIndicator status;
				std::string statusString;
				rclcpp::Publisher<task_manager_msgs::msg::TaskStatus>::SharedPtr statusPub;
				
				ThreadParameters(rclcpp::Publisher<task_manager_msgs::msg::TaskStatus>::SharedPtr pub,
                        TaskScheduler *ts, 
						TaskDefinitionPtr td, 
						double tperiod);
				ThreadParameters(const ThreadParameters & tp);
				~ThreadParameters();

				task_manager_msgs::msg::TaskStatus getRosStatus() {
					task_manager_msgs::msg::TaskStatus st = task->getRosStatus();
					st.id = tpid;
					st.status = status | 
						(foreground?TASK_FOREGROUND:TASK_BACKGROUND);
					st.status_time = statusTime;
                    params.exportToMessage(st.plist);
					return st;
				}
				
				
                bool isAnInstanceOf(const TaskDefinitionPtr & def) {
                    return task->isAnInstanceOf(def);
                }


				void setStatus(TaskIndicator newstatus, const std::string & text, const rclcpp::Time & tnow) {
                    task->setStatus(newstatus);
                    task->setStatusString(text);
                    updateStatus(tnow);
				}

				void updateStatus(const rclcpp::Time & tnow) {
					status = task->getStatus();
					statusString = task->getStatusString();
					statusTime = tnow;
					statusPub->publish(getRosStatus());
					// manageHistory();
					// task->debug("Pub: task %d %s %s",tpid, task->getName().c_str(),taskStatusToString(status));
				}

				
#if 0
				void manageHistory()
				{
					if (status==task_manager_msgs::msg::TaskStatus::TASK_INITIALISED)
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
					
					if (status>=task_manager_msgs::msg::TaskStatus::TASK_RUNNING )
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
#endif
				
				bool operator<(const ThreadParameters & tp) {
					return tpid < tp.tpid;
				}
			};
			
			
			
		protected:
            std::shared_ptr<rclcpp::Node> node;
			rclcpp::Time lastKeepAlive;
			rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr keepAliveSub;
			rclcpp::Publisher<task_manager_msgs::msg::TaskStatus>::SharedPtr statusPub;
			rclcpp::Service<task_manager_msgs::srv::StartTask>::SharedPtr startTaskSrv;
			rclcpp::Service<task_manager_msgs::srv::StopTask>::SharedPtr stopTaskSrv;
			rclcpp::Service<task_manager_msgs::srv::GetTaskList>::SharedPtr getTaskListSrv;
			rclcpp::Service<task_manager_msgs::srv::GetAllTaskStatus>::SharedPtr getAllTaskStatusSrv;
#if 0
            // Not implemented for ROS2
			rclcpp::Client<> getTaskListLightSrv;
			rclcpp::Client<> getHistorySrv;
			rclcpp::Client<> executeSequenceTasksSrv;
			std::vector<TaskHistory> history;
#endif
			void keepAliveCallback(const std_msgs::msg::Header::ConstSharedPtr msg);

		protected:
			TaskDefinitionPtr idle;
			typedef std::map<std::string,TaskDefinitionPtr,std::less<std::string> > TaskDirectory;
			typedef std::pair<unsigned int, std::shared_ptr<ThreadParameters> > TaskSetItem;
			typedef std::map<unsigned int, std::shared_ptr<ThreadParameters>, std::less<unsigned int> > TaskSet;
			TaskDirectory tasks;
			TaskSet runningThreads,zombieThreads;
			static void printTaskSet(const std::string & name, const TaskSet & ts);
			std::shared_ptr<ThreadParameters> mainThread;

		protected:
			static const rclcpp::Duration IDLE_TIMEOUT;
			static const rclcpp::Duration DELETE_TIMEOUT;
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
				std::shared_ptr<ThreadParameters> tp;
                ThreadAction() : type(NO_ACTION) {}
			};
			static const char * actionString(ActionType at);

			// Action queueing mecanism. Action are used to manage the threads
			// associated with each task.
			typedef std::map<double, ThreadAction, std::less<double> > ActionQueue;
			ActionQueue actionQueue;
			ThreadAction getNextAction();
			void enqueueAction(ActionType type,std::shared_ptr<ThreadParameters> tp);
			void enqueueAction(const rclcpp::Time & when, ActionType type,std::shared_ptr<ThreadParameters> tp);
			void removeConditionalIdle();
			bool runScheduler;
            boost::thread aqid;
            std::mutex aqMutex;
            std::condition_variable aqCond;
			int runSchedulerLoop();


		protected:
			double idleTimeout;
			double startingTime;
			double defaultPeriod;
            std::mutex scheduler_mutex;
            std::condition_variable scheduler_condition;

			TaskId launchTask(std::shared_ptr<ThreadParameters> tp);
			void runTask(std::shared_ptr<ThreadParameters> tp);
			void runAperiodicTask(std::shared_ptr<ThreadParameters> tp);
			void terminateTask(std::shared_ptr<ThreadParameters> tp);
			void cleanupTask(std::shared_ptr<ThreadParameters> tp);
			void deleteTask(std::shared_ptr<ThreadParameters> tp);

			// All ROS callbacks
			bool startTask(const std::shared_ptr<task_manager_msgs::srv::StartTask::Request>  req,
					const std::shared_ptr<task_manager_msgs::srv::StartTask::Response> res );
			bool stopTask(const std::shared_ptr<task_manager_msgs::srv::StopTask::Request>  req,
					const std::shared_ptr<task_manager_msgs::srv::StopTask::Response> res );
			bool getAllTaskStatus(const std::shared_ptr<task_manager_msgs::srv::GetAllTaskStatus::Request>  req,
					const std::shared_ptr<task_manager_msgs::srv::GetAllTaskStatus::Response> res );
			bool getTaskList(const std::shared_ptr<task_manager_msgs::srv::GetTaskList::Request>  req,
					const std::shared_ptr<task_manager_msgs::srv::GetTaskList::Response> res );
			// Generate a task description list, ready to be published by ROS
			void generateTaskList(std::vector<task_manager_msgs::msg::TaskDescription> & tlist) const;
			// Generate a task status vector, ready to be published by ROS
			void generateTaskStatus(std::vector<task_manager_msgs::msg::TaskStatus> & running,
					std::vector<task_manager_msgs::msg::TaskStatus> & zombies) ;

            
            rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr setParamHandle;
            rcl_interfaces::msg::SetParametersResult
                reconfigure_callback(const std::vector<rclcpp::Parameter> & parameters);


#if 0
            // Not implemented for ROS2
			bool getTaskListLight(task_manager_lib::GetTaskListLight::Request  &req, task_manager_lib::GetTaskListLight::Response &res );
			bool getHistory(task_manager_lib::GetHistory::Request  &req, task_manager_lib::GetHistory::Response &res );
			bool executeTaskSequence(task_manager_lib::ExeTaskSequence::Request  &req,task_manager_lib::ExeTaskSequence::Response &res);
			void generateTaskListLight(std::vector<task_manager_msgs::TaskDescription> &input,std::vector<task_manager_msgs::TaskDescriptionLight> &output) const;
			void generateHistory(std::vector<task_manager_msgs::TaskHistory> &output) ;
			void launchTaskSequence(std::vector<task_manager_msgs::TaskDescriptionLight> &tasks, int & id);
			void DescriptionLightToTaskParameters(const task_manager_msgs::TaskDescriptionLight& task_params, TaskParameters& output);
#endif


			// convenience function
			rclcpp::Time now() { return node->get_clock()->now();}
			
		public:
			// Default constructor:
			// nh: ros NodeHandle in which all the services and topic will  be
			// created.
			// idle: a pointer to the definition of the idle class
			// deftPeriod: the default period for task execution
			TaskScheduler(std::shared_ptr<rclcpp::Node> node, TaskDefinitionPtr idle, double deftPeriod);
			~TaskScheduler();

			// Cleanup all the task
			int terminateAllTasks();
			// Print the task directory (added task plus dynamic tasks)
			void printTaskDirectory(bool with_ros=true) const;
			// Return the task directory
			const TaskDirectory & getDirectory() const {return tasks;}

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
			void addTask(TaskDefinitionPtr task);

			// Load a task from a file to the directory, and create it with
			// argument env
			void loadTask(const std::string & filename, TaskEnvironmentPtr env);
			// Load all task from a folder and initialise all of them with env.
			void loadAllTasks(const std::string & dirname, TaskEnvironmentPtr env);

            // Remove all the dynamic tasks from the list of known tasks
            void clearAllDynamicTasks();
			
			double getDfltPeriod(){return defaultPeriod;};
			
			int getStatus(unsigned int &taskid);
			
			int terminateTask(unsigned int &taskid);
			
			void keepAliveSequence();
			
            std::shared_ptr<rclcpp::Node> getNodeHandle() {
                return node;
            }
			
			
	};


}


#endif // TASK_SCHEDULER_H
