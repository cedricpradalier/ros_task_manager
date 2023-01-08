
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_sync/TaskEnvironmentSync.h>


using namespace task_manager_sync;

void TaskEnvironmentSync::syncTimerCallback() {
    task_manager_msgs::msg::SyncStatus status;
    status.header.stamp = node->get_clock()->now();
    status.header.frame_id = myName;
    status.status = myStatus;
    myStatusPub->publish(status);
}


TaskEnvironmentSync::TaskEnvironmentSync(std::shared_ptr<rclcpp::Node> n, const std::string & name,
        const std::string & topic, float period) : task_manager_lib::TaskEnvironment(n), myName(name), myStatus(0){
    using namespace std::chrono_literals;
    myStatusPub = node->create_publisher<task_manager_msgs::msg::SyncStatus>(topic,1);
    syncTimer = node->create_wall_timer(int(period*1000)*1ms, 
            std::bind(&TaskEnvironmentSync::syncTimerCallback,this));
}

void TaskEnvironmentSync::addSyncSource(const std::string & topic) {
    auto sub = node->create_subscription<task_manager_msgs::msg::SyncStatus>(topic,1,
            std::bind(&TaskEnvironmentSync::syncCallback,this,std::placeholders::_1));
    syncSubscriptions.push_back(sub);
}


int TaskEnvironmentSync::getStatus(const std::string & sourceName, int defaultvalue) const {
    std::map<std::string,task_manager_msgs::msg::SyncStatus>::const_iterator it =
        syncStatus.find(sourceName);
    if (it == syncStatus.end()) {
        return defaultvalue;
    } else {
        return it->second.status;
    }
}

bool TaskEnvironmentSync::getStatusRef(const std::string & sourceName, int & status, rclcpp::Time & stamp) const {
    std::map<std::string,task_manager_msgs::msg::SyncStatus>::const_iterator it = 
        syncStatus.find(sourceName);
    if (it == syncStatus.end()) {
        return false;
    } else {
        status = it->second.status;
        stamp = it->second.header.stamp;
        return true;
    }
}


bool TaskEnvironmentSync::getStatusRef(const std::string & sourceName, int & status) const {
    std::map<std::string,task_manager_msgs::msg::SyncStatus>::const_iterator it =
        syncStatus.find(sourceName);
    if (it == syncStatus.end()) {
        return false;
    } else {
        status = it->second.status;
        return true;
    }
}


