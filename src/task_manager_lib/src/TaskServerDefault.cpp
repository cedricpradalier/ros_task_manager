

#include <memory>
#include "task_manager_lib/TaskServerDefault.h"

using namespace task_manager_lib; 


void TaskServerBase::reloadTasks() {
    RCLCPP_INFO(this->get_logger(), "Terminating all tasks");
    ts.terminateAllTasks();
    ts.stopScheduler();
    RCLCPP_INFO(this->get_logger(), "Clearing all dynamic tasks");
    ts.clearAllDynamicTasks();
    RCLCPP_INFO(this->get_logger(), "Reloading all dynamic tasks");
    ts.loadAllTasks(lib_path,env);
    ts.printTaskDirectory(true);
    ts.startScheduler();
    RCLCPP_INFO(this->get_logger(), "Reload completed");
}

TaskServerBase::TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, bool default_wait) : rclcpp::Node(name), lib_path("./lib"), env(_env), 
    idle(new TaskFactoryIdleDefault(env)), ts(shared_from_this(), idle, 0.5)/*, tsi(ts)*/ {
        this->declare_parameter("lib_path", lib_path);
        if (default_wait) {
            TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
            ts.addTask(wait);
        }
    }

TaskServerBase::TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, bool default_wait) : rclcpp::Node(name), lib_path("./lib"), 
    env(_env), idle(_idle), ts(shared_from_this(), idle, 0.5)/*, tsi(ts)*/ {
        this->declare_parameter("lib_path", lib_path);
        if (default_wait) {
            TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
            ts.addTask(wait);
        }
    }

TaskServerBase::TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, TaskDefinitionPtr _wait) : rclcpp::Node(name), lib_path("./lib"), 
    env(_env), idle(_idle), ts(shared_from_this(), idle, 0.5)/*, tsi(ts)*/ {
        this->declare_parameter("lib_path", lib_path);
        if (_wait) {
            ts.addTask(_wait);
        }
    }

void TaskServerBase::start() {
    lib_path = this->get_parameter("lib_path").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Looking for tasks in %s",lib_path.c_str());
    service = this->create_service<std_srvs::srv::Empty>("reload_tasks", std::bind(&TaskServerBase::reloadSrv,this,std::placeholders::_1,std::placeholders::_2));
    ts.loadAllTasks(lib_path,env);
    ts.printTaskDirectory(true);
    ts.startScheduler();
}



