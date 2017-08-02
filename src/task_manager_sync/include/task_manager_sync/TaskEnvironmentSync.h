#ifndef TASK_MANAGER_SYNC_ENV_H
#define TASK_MANAGER_SYNC_ENV_H

#include <ros/ros.h>
#include <task_manager_lib/TaskDefinition.h>
#include <task_manager_sync/SyncStatus.h>


namespace task_manager_sync {
    class TaskEnvironmentSync: public task_manager_lib::TaskEnvironment
    {
        protected:
            std::string myName;

            ros::Publisher myStatusPub;
            ros::Timer syncTimer;
            int myStatus;

            std::map<std::string,task_manager_sync::SyncStatus> syncStatus;

            void syncCallback(const task_manager_sync::SyncStatus::ConstPtr& msg) {
                // Assuming a synchronous spinner
                syncStatus[msg->header.frame_id] = *msg;
            }

            std::vector<ros::Subscriber> syncCallbacks;
            
            void syncTimerCallback(const ros::TimerEvent&) {
                task_manager_sync::SyncStatus status;
                status.header.stamp = ros::Time::now();
                status.header.frame_id = myName;
                status.status = myStatus;
                myStatusPub.publish(status);
            }


        public:
            TaskEnvironmentSync(ros::NodeHandle & n, const std::string & name,
                    const std::string & topic, float period=0.1) : task_manager_lib::TaskEnvironment(n), myName(name), myStatus(0){
                myStatusPub = nh.advertise<task_manager_sync::SyncStatus>(topic,10);
                syncTimer = nh.createTimer(ros::Duration(period), &TaskEnvironmentSync::syncTimerCallback,this);
            }
            virtual ~TaskEnvironmentSync() {};

            void addSyncSource(const std::string & topic) {
                // This relies on the proper behaviour of the sub
                // copy-constructor
                syncCallbacks.push_back(getNodeHandle().subscribe(topic,10,&TaskEnvironmentSync::syncCallback,this));
            }

            void setStatus(int status) {
				myStatus = status;
			}
                      
            int getStatus(const std::string & sourceName, int defaultvalue=-1) const {
                std::map<std::string,task_manager_sync::SyncStatus>::const_iterator it = syncStatus.find(sourceName);
                if (it == syncStatus.end()) {
                    return defaultvalue;
                } else {
                    return it->second.status;
                }
            }

            bool getStatusRef(const std::string & sourceName, int & status, ros::Time & stamp) const {
                std::map<std::string,task_manager_sync::SyncStatus>::const_iterator it = syncStatus.find(sourceName);
                if (it == syncStatus.end()) {
                    return false;
                } else {
                    status = it->second.status;
                    stamp = it->second.header.stamp;
                    return true;
                }
            }

            
            bool getStatusRef(const std::string & sourceName, int & status) const {
                std::map<std::string,task_manager_sync::SyncStatus>::const_iterator it = syncStatus.find(sourceName);
                if (it == syncStatus.end()) {
                    return false;
                } else {
                    status = it->second.status;
                    return true;
                }
            }
            
            // To be redefined in inherited class
            virtual bool isStatusValidForMe(int status) const {
                return true;
            }

            virtual bool isStatusValidForSource(const std::string & sourceName, int status) const {
                return true;
            }

    };

    typedef boost::shared_ptr<TaskEnvironmentSync> TaskEnvironmentSyncPtr;
    typedef boost::shared_ptr<TaskEnvironmentSync const> TaskEnvironmentSyncConstPtr;
};

#endif // TASK_MANAGER_SYNC_ENV_H

