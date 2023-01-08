#ifndef TASK_MANAGER_SYNC_ENV_H
#define TASK_MANAGER_SYNC_ENV_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_lib/TaskEnvironment.h>
#include <task_manager_msgs/msg/sync_status.hpp>


namespace task_manager_sync {
    class TaskEnvironmentSync: public task_manager_lib::TaskEnvironment
    {
        protected:
            std::string myName;

            rclcpp::Publisher<task_manager_msgs::msg::SyncStatus>::SharedPtr myStatusPub;
            rclcpp::TimerBase::SharedPtr syncTimer;
            int myStatus;

            std::map<std::string,task_manager_msgs::msg::SyncStatus> syncStatus;

            void syncCallback(const task_manager_msgs::msg::SyncStatus::SharedPtr msg) {
                // Assuming a synchronous spinner
                syncStatus[msg->header.frame_id] = *msg;
            }

            std::vector<rclcpp::Subscription<task_manager_msgs::msg::SyncStatus>::SharedPtr> syncSubscriptions;
            
            void syncTimerCallback() ;


        public:
            TaskEnvironmentSync(std::shared_ptr<rclcpp::Node> n, const std::string & name,
                    const std::string & topic="~/sync", float period=0.1);
            virtual ~TaskEnvironmentSync() {};

            void addSyncSource(const std::string & topic) ;

            void setStatus(int status) {
				myStatus = status;
			}
                      
            int getStatus(const std::string & sourceName, int defaultvalue=-1) const ;

            bool getStatusRef(const std::string & sourceName, int & status, rclcpp::Time & stamp) const ;

            
            bool getStatusRef(const std::string & sourceName, int & status) const ;
            
            // To be redefined in inherited class
            virtual bool isStatusValidForMe(int status) const {
                (void)status;
                return true;
            }

            virtual bool isStatusValidForSource(const std::string & sourceName, int status) const {
                (void)sourceName;
                (void)status;
                return true;
            }

    };

    typedef std::shared_ptr<TaskEnvironmentSync> TaskEnvironmentSyncPtr;
    typedef std::shared_ptr<TaskEnvironmentSync const> TaskEnvironmentSyncConstPtr;
}

#endif // TASK_MANAGER_SYNC_ENV_H

