#ifndef TASK_CLIENT_H
#define TASK_CLIENT_H

#include <vector>
#include <string>
#include <map>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/Config.h>
#include "task_manager_lib/TaskScheduler.h"

namespace task_manager_lib {
    // Class to interact with a remote task scheduler, get the task list, task
    // status, start and stop tasks.
    // In most cases, the python TaskClient is more practical
    class TaskClient {
        public:

            // Local struct to keep track of the task status.
            // Will be updated as data is published by the scheduler
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
            mutable boost::mutex mutex;
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

            // Set of classes to implement a keep-alive connection from the client
            // to the scheduler. If the scheduler does not receive the messages
            // from the client for more than 1s, it stops the current task
            bool keepAlive;
            ros::Timer keepAliveTimer;
            ros::Publisher keepAlivePub;
            void timerCallback(const ros::TimerEvent&);
        public:
            // Default constructor: 
            // node: node name of the task scheduler
            // nh: ros node handle in which the different ROS objects will be
            // created. All topics and services are connected with absolute names.
            TaskClient(const std::string & node, ros::NodeHandle & nh);
            ~TaskClient();

            // Use a service to request the task list and description from the scheduler
            // This is called at the end of the constructor, and it should not be
            // needed later.
            int updateTaskList();
            // Returns the task list
            const std::vector<task_manager_msgs::TaskDescription> & getTaskList() const;
            // Print the task list.
            void printTaskList() const;

            // Start a task based on its name and the task parameters
            TaskScheduler::TaskId startTask(const std::string & taskname, 
                    const TaskParameters & tp);
            // Start a task based on its name and other flags and the task
            // parameters.
            TaskScheduler::TaskId startTask(const std::string & taskname, 
                    bool foreground, double period,
                    const TaskParameters & tp);

            // Call startTask then waitTask. Useful for foreground tasks.
            bool startTaskAndWait(const std::string & taskname, 
                    const TaskParameters & tp);
            // SWitch back to idle, eventually terminating the current task
            int idle();

            // Wait for task tid to complete
            bool waitTask(TaskScheduler::TaskId tid);

            // Get the updated status map.
            const StatusMap & getStatusMap() const {
                return taskStatus;
            }
            // Use the ros service to request the status of all the tasks currently
            // in scope.
            void updateAllStatus();
            // Print the status map
            void printStatusMap() const;
    };

};

#endif // TASK_CLIENT_H
