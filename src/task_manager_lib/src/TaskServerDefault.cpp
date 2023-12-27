

#include <memory>
#include "task_manager_lib/TaskServerDefault.h"
#include "task_manager_lib/TaskSystem.h"

using namespace task_manager_lib; 


void TaskServerBase::reloadTasks() {
    RCLCPP_INFO(node->get_logger(), "Terminating all tasks");
    ts.terminateAllTasks();
    ts.stopScheduler();
    RCLCPP_INFO(node->get_logger(), "Clearing all dynamic tasks");
    ts.clearAllDynamicTasks();
    RCLCPP_INFO(node->get_logger(), "Reloading all dynamic tasks");
    ts.loadAllTasks(lib_path,env);
    ts.printTaskDirectory(true);
    ts.startScheduler();
    RCLCPP_INFO(node->get_logger(), "Reload completed");
}

TaskServerBase::TaskServerBase(TaskEnvironmentPtr _env, bool default_wait) : node(_env->getNode()), lib_path("./lib"), env(_env), 
    idle(new TaskFactoryIdleDefault(env)), ts(node, idle, 0.5)/*, tsi(ts)*/ {
        node->declare_parameter("lib_path", lib_path);
        if (default_wait) {
            TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
            ts.addTask(wait);
        }
    }

TaskServerBase::TaskServerBase(TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, bool default_wait) : node(_env->getNode()), lib_path("./lib"), 
    env(_env), idle(_idle), ts(node, idle, 0.5)/*, tsi(ts)*/ {
        node->declare_parameter("lib_path", lib_path);
        if (default_wait) {
            TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
            ts.addTask(wait);
        }
    }

TaskServerBase::TaskServerBase(TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, TaskDefinitionPtr _wait) : node(_env->getNode()), lib_path("./lib"), 
    env(_env), idle(_idle), ts(node, idle, 0.5)/*, tsi(ts)*/ {
        node->declare_parameter("lib_path", lib_path);
        if (_wait) {
            ts.addTask(_wait);
        }
    }

void TaskServerBase::start() {
    lib_path = node->get_parameter("lib_path").get_parameter_value().get<std::string>();

    RCLCPP_INFO(node->get_logger(), "Looking for tasks in %s",lib_path.c_str());
    service = node->create_service<std_srvs::srv::Empty>("reload_tasks", std::bind(&TaskServerBase::reloadSrv,this,std::placeholders::_1,std::placeholders::_2));
    ts.loadAllTasks(lib_path,env);
    ts.printTaskDirectory(true);
    ts.startScheduler();
}

void TaskServerBase::addSystemTask() {
    TaskDefinitionPtr sys(new TaskFactorySystem(env));
    ts.addTask(sys);
}


