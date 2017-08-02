#ifndef TASK_SERVER_DEFAULT_H
#define TASK_SERVER_DEFAULT_H

#include <std_srvs/Empty.h>

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/TaskServerInterface.h"
#include "task_manager_lib/TaskIdleDefault.h"
#include "task_manager_lib/TaskWaitDefault.h"


namespace task_manager_lib {

    class TaskServerBase {
        protected:
            ros::NodeHandle nh;
            ros::ServiceServer service;
            std::string lib_path;
            TaskEnvironmentPtr env;
            TaskDefinitionPtr idle;
            TaskDefinitionPtr wait;
            TaskScheduler ts;
            TaskServerInterface tsi;

            void reloadTasks() {
                ROS_INFO("Terminating all tasks");
                ts.terminateAllTasks();
                ts.stopScheduler();
                ROS_INFO("Clearing all dynamic tasks");
                ts.clearAllDynamicTasks();
                ROS_INFO("Reloading all dynamic tasks");
                ts.loadAllTasks(lib_path,env);
                ts.configureTasks();
                ts.printTaskDirectory(true);
                ts.startScheduler();
                ROS_INFO("Reload completed");
            }

            bool reloadSrv(std_srvs::Empty::Request  &req,
                    std_srvs::Empty::Response &res)
            {
                reloadTasks();
                return true;
            }

        public:
            TaskServerBase(TaskEnvironmentPtr _env, bool default_wait=false) : nh("~"), lib_path("./lib"), env(_env), 
            idle(new TaskFactoryIdleDefault(env)), ts(nh, idle, 0.5), tsi(ts) {
                if (default_wait) {
                    TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
                    ts.addTask(wait);
                }
            }

            TaskServerBase(TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, bool default_wait=false) : nh("~"), lib_path("./lib"), 
            env(_env), idle(_idle), ts(nh, idle, 0.5), tsi(ts) {
                if (default_wait) {
                    TaskDefinitionPtr wait(new TaskFactoryWaitDefault(env));
                    ts.addTask(wait);
                }
            }

            TaskServerBase(TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, TaskDefinitionPtr _wait) : nh("~"), lib_path("./lib"), 
            env(_env), idle(_idle), ts(nh, idle, 0.5), tsi(ts) {
                if (_wait) {
                    ts.addTask(_wait);
                }
            }

            void start() {
                nh.getParam("lib_path",lib_path);
                ROS_INFO("Looking for tasks in %s",lib_path.c_str());
                service = nh.advertiseService("reload_tasks", &TaskServerBase::reloadSrv,this);
                ts.loadAllTasks(lib_path,env);
                ts.configureTasks();
                ts.printTaskDirectory(true);
                ts.startScheduler();
            }


			void addTask(boost::shared_ptr<TaskDefinitionBase> task) {
                ts.addTask(task);
            }


    };

};

#endif // TASK_SERVER_DEFAULT_H
