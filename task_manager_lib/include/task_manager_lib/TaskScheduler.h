#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <vector>
#include <set>
#include <map>

#include "TaskDefinition.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "task_manager_lib/StartTask.h"
#include "task_manager_lib/StopTask.h"
#include "task_manager_lib/GetTaskList.h"
#include "task_manager_lib/GetAllTaskStatus.h"
#include "task_manager_msgs/TaskStatus.h"

#define TASK_STATUS_MASK 0xFFl
#define TASK_FOREGROUND 0x100l
#define TASK_BACKGROUND 0x000l

// Class that manages the execution of task directly or through ROS services
class TaskScheduler
{
	public:
		typedef unsigned int TaskId;
	protected:
		struct ThreadParameters {
			static unsigned int gtpid;

			bool foreground,running;
			unsigned int tpid;
            boost::shared_ptr<TaskDefinition> task;
            TaskParameters params;
			TaskScheduler *that;
			double period;
			pthread_t tid;
			pthread_cond_t task_condition;
			pthread_mutex_t task_mutex;

			pthread_cond_t aperiodic_task_condition;
			pthread_mutex_t aperiodic_task_mutex;

            ros::Time statusTime;
			TaskIndicator status;
			std::string statusString;
            ros::Publisher statusPub;

			ThreadParameters(ros::Publisher pub, TaskScheduler *ts, 
					boost::shared_ptr<TaskDefinition> td, 
					double tperiod);
			ThreadParameters(const ThreadParameters & tp);
			~ThreadParameters();

            task_manager_msgs::TaskStatus getRosStatus() const {
                task_manager_msgs::TaskStatus st = task->getRosStatus();
                st.id = tpid;
                st.status = status | 
                    (foreground?TASK_FOREGROUND:TASK_BACKGROUND);
                st.status_time = statusTime;
                return st;
            }
				

			void setStatus(TaskIndicator newstatus, const std::string & text, const ros::Time & tnow) {
				status = newstatus;
				statusString = text;
				statusTime = tnow;
                statusPub.publish(getRosStatus());
			}
				
			void updateStatus(const ros::Time & tnow) {
				status = task->getStatus();
				statusString = task->getStatusString();
				statusTime = tnow;
                statusPub.publish(getRosStatus());
                // ROS_INFO("Pub: task %d %s %s",tpid, task->getName().c_str(),taskStatusToString(status));
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

        void keepAliveCallback(const std_msgs::Header::ConstPtr& msg);

	protected:


        boost::shared_ptr<TaskDefinition> idle;
		typedef std::map<std::string,boost::shared_ptr<TaskDefinition>,std::less<std::string> > TaskDirectory;
		typedef std::pair<unsigned int, boost::shared_ptr<ThreadParameters> > TaskSetItem;
		typedef std::map<unsigned int, boost::shared_ptr<ThreadParameters>, std::less<unsigned int> > TaskSet;
		TaskDirectory tasks;

		TaskSet runningThreads,zombieThreads;
		static void printTaskSet(const std::string & name, const TaskSet & ts);

        boost::shared_ptr<ThreadParameters> mainThread;

	protected:
		static unsigned int debug;
		static const double IDLE_TIMEOUT;
		static const double DELETE_TIMEOUT;

		// Action management: create task, terminate task
		typedef enum {
			START_IDLE_TASK, 
			START_TASK,
			CONDITIONALLY_IDLE,
			DELETE_TASK,
			WAIT_CANCELLED
		} ActionType;
		struct ThreadAction {
			ActionType type;
            boost::shared_ptr<ThreadParameters> tp;
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
		pthread_t aqid;
		pthread_mutex_t aqMutex;
		pthread_cond_t aqCond;
		static void cleanup_action(void*);
		static void * scheduler_thread(void*);
		int runSchedulerLoop();


	protected:
		double idleTimeout;
		double startingTime;
		double defaultPeriod;
		pthread_mutex_t scheduler_mutex;
		pthread_cond_t scheduler_condition;

        // Basic thread management functions
		static void * thread_func(void *);
		static void cleanitup(void * arg);
		static void* runAperiodicTask(void * arg);
		TaskId launchTask(boost::shared_ptr<ThreadParameters> tp);
		int runTask(boost::shared_ptr<ThreadParameters> tp);
		int terminateTask(boost::shared_ptr<ThreadParameters> tp);
		int cleanupTask(boost::shared_ptr<ThreadParameters> tp);
		int deleteTask(boost::shared_ptr<ThreadParameters> tp);

        // Wrapper around pthread_mutex_lock/unlock
		void lockScheduler();
		void unlockScheduler();

        // All ROS callbacks
        bool startTask(task_manager_lib::StartTask::Request  &req,
                task_manager_lib::StartTask::Response &res );
        bool stopTask(task_manager_lib::StopTask::Request  &req,
                task_manager_lib::StopTask::Response &res );
        bool getAllTaskStatus(task_manager_lib::GetAllTaskStatus::Request  &req,
                task_manager_lib::GetAllTaskStatus::Response &res );
        bool getTaskList(task_manager_lib::GetTaskList::Request  &req,
                task_manager_lib::GetTaskList::Response &res );

        // Generate a task description list, ready to be published by ROS
		void generateTaskList(std::vector<task_manager_msgs::TaskDescription> & tlist) const;
        // Generate a task status vector, ready to be published by ROS
		void generateTaskStatus(std::vector<task_manager_msgs::TaskStatus> & running,
                std::vector<task_manager_msgs::TaskStatus> & zombies) ;

        // convenience function
        ros::Time now() { return ros::Time::now();}
	public:
        // Default constructor:
        // nh: ros NodeHandle in which all the services and topic will  be
        // created.
        // idle: a pointer to the definition of the idle class
        // deftPeriod: the default period for task execution
		TaskScheduler(ros::NodeHandle & nh, boost::shared_ptr<TaskDefinition> idle, double deftPeriod);
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
		void addTask(boost::shared_ptr<TaskDefinition> task);

        // Load a task from a file to the directory, and create it with
        // argument env
		void loadTask(const std::string & filename, boost::shared_ptr<TaskEnvironment> env);
        // Load all task from a folder and initialise all of them with env.
		void loadAllTasks(const std::string & dirname, boost::shared_ptr<TaskEnvironment> env);

};




#endif // TASK_SCHEDULER_H
