#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <vector>
#include <set>
#include <map>

#include "TaskDefinition.h"
#include <ros/ros.h>
#include "task_manager_lib/StartTask.h"
#include "task_manager_lib/StopTask.h"
#include "task_manager_lib/GetTaskList.h"
#include "task_manager_lib/GetAllTaskStatus.h"
#include "task_manager_msgs/TaskStatus.h"

#define TASK_STATUS_MASK 0xFFl
#define TASK_FOREGROUND 0x100l
#define TASK_BACKGROUND 0x000l

class TaskScheduler
{
	public:
		typedef unsigned int TaskId;
	protected:
		struct ThreadParameters {
			static unsigned int gtpid;

			bool foreground,running;
			unsigned int tpid;
			TaskDefinition *task;
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
					TaskDefinition *td, 
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
        ros::Publisher statusPub;
        ros::ServiceServer startTaskSrv;
        ros::ServiceServer stopTaskSrv;
        ros::ServiceServer getTaskListSrv;
        ros::ServiceServer getAllTaskStatusSrv;

	protected:


		TaskDefinition *idle;
		typedef std::map<std::string,TaskDefinition*,std::less<std::string> > TaskDirectory;
		typedef std::pair<unsigned int, ThreadParameters*> TaskSetItem;
		typedef std::map<unsigned int, ThreadParameters*, std::less<unsigned int> > TaskSet;
		TaskDirectory tasks;

		TaskSet runningThreads,zombieThreads;
		static void printTaskSet(const std::string & name, const TaskSet & ts);

		ThreadParameters *mainThread;

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
			ThreadParameters *tp;
		};
		static const char * actionString(ActionType at);

		
		typedef std::map<double, ThreadAction, std::less<double> > ActionQueue;
		ActionQueue actionQueue;
		ThreadAction getNextAction();
		void enqueueAction(ActionType type,ThreadParameters * tp);
		void enqueueAction(const ros::Time & when, ActionType type,ThreadParameters * tp);
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

		static void * thread_func(void *);
		static void cleanitup(void * arg);
		static void* runAperiodicTask(void * arg);
		TaskId launchTask(ThreadParameters *tp);
		int runTask(ThreadParameters *tp);
		int terminateTask(ThreadParameters *tp);
		int cleanupTask(ThreadParameters *tp);
		int deleteTask(ThreadParameters *tp);

		void lockScheduler();
		void unlockScheduler();
        bool startTask(task_manager_lib::StartTask::Request  &req,
                task_manager_lib::StartTask::Response &res );
        bool stopTask(task_manager_lib::StopTask::Request  &req,
                task_manager_lib::StopTask::Response &res );
        bool getAllTaskStatus(task_manager_lib::GetAllTaskStatus::Request  &req,
                task_manager_lib::GetAllTaskStatus::Response &res );
        bool getTaskList(task_manager_lib::GetTaskList::Request  &req,
                task_manager_lib::GetTaskList::Response &res );

	public:
		TaskScheduler(ros::NodeHandle & nh, TaskDefinition *idle, double deftPeriod);
		~TaskScheduler();
		int terminateAllTasks();
		void printTaskDirectory() const;
		void configureTasks(/* const std::string & dirname=".", const std::string & extension="cfg" */);

		int startScheduler();
		int stopScheduler();
        ros::Time now() { return ros::Time::now();}

		const TaskDirectory & getDirectory() const {return tasks;}

		TaskId launchTask(const std::string & taskname, const TaskParameters & tp);
		TaskId launchIdleTask();
		int waitTaskCompletion(TaskId id, double timeout);


		void addTask(TaskDefinition *task);

		void loadTask(const std::string & filename, TaskEnvironment *env);
		void loadAllTasks(const std::string & dirname, TaskEnvironment *env);


		void generateTaskList(std::vector<task_manager_msgs::TaskDescription> & tlist) const;
		void generateTaskStatus(std::vector<task_manager_msgs::TaskStatus> & running,
                std::vector<task_manager_msgs::TaskStatus> & zombies) ;

};




#endif // TASK_SCHEDULER_H
