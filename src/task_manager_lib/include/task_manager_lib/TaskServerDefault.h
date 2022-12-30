#ifndef TASK_SERVER_DEFAULT_H
#define TASK_SERVER_DEFAULT_H


#include <std_srvs/srv/empty.hpp>
#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/TaskIdleDefault.h"
#include "task_manager_lib/TaskWaitDefault.h"
// Not implemented for ROS2
//#include "task_manager_lib/TaskServerInterface.h"


namespace task_manager_lib {

    class TaskServerBase : public rclcpp::Node {
        protected:
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
            std::string lib_path;
            TaskEnvironmentPtr env;
            TaskDefinitionPtr idle;
            TaskDefinitionPtr wait;
            TaskScheduler ts;
            // TaskServerInterface tsi;

            void reloadTasks() ;

            bool reloadSrv(std::shared_ptr<std_srvs::srv::Empty::Request> ,
                    std::shared_ptr<std_srvs::srv::Empty::Response>)
            {
                reloadTasks();
                return true;
            }

        public:
            TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, bool default_wait=false);

            TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, bool default_wait=false);

            TaskServerBase(const std::string & name, TaskEnvironmentPtr _env, TaskDefinitionPtr _idle, TaskDefinitionPtr _wait);

            void start() ;


			void addTask(TaskDefinitionPtr task) {
                ts.addTask(task);
            }

    };

}

#endif // TASK_SERVER_DEFAULT_H
