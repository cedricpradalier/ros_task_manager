#ifndef TASK_SET_BOOL_H
#define TASK_SET_BOOL_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_lib/TaskServiceGeneric.h"
#include "std_srvs/srv/set_bool.hpp"

namespace task_manager_lib {
    struct TaskSetBoolConfig : public TaskServiceGenericConfig {
        TaskSetBoolConfig() : TaskServiceGenericConfig("/service_name") {
            define("value",false,"value of the boolean",true);
        }
    };

    class TaskSetBool : public TaskServiceGeneric<std_srvs::srv::SetBool,TaskSetBoolConfig,TaskEnvironment>
    {
        protected:
            virtual void buildServiceRequest(Request& req) {
                req.data=cfg->get<bool>("value");
                RCLCPP_INFO(node->get_logger(),"Setting bool to %s",
                        req.data?"true":"false");
            }

        public:
            TaskSetBool(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetBool() {};
    };

    class TaskFactorySetBool : public TaskDefinition<TaskSetBoolConfig, TaskEnvironment, TaskSetBool>
    {

        public:
            TaskFactorySetBool(TaskEnvironmentPtr env) : 
                Parent("SetBool","Use a service to set a bool",true,env) {}
            virtual ~TaskFactorySetBool() {};
    };
}

#endif // TASK_SET_BOOL_H
