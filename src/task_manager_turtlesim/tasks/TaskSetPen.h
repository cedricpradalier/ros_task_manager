#ifndef TASK_SETPEN_H
#define TASK_SETPEN_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskSetPenConfig : public TaskServiceGenericWithoutClientConfig {
        TaskSetPenConfig() : TaskServiceGenericWithoutClientConfig() {
            define("r",128,"Red component",true); 
            define("g",128,"Green component",true); 
            define("b",128,"Blue component",true); 
            define("width",1,"Line width",true); 
            define("on",true,"Activate pen",true); 
        }
    };

    class TaskSetPen : public TaskServiceGenericWithoutClient<turtlesim::srv::SetPen,TaskSetPenConfig,TurtleSimEnv>
    {
        protected:
            virtual rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr getServiceClient() {
                return env->getSetPenClient();
            }
            virtual void buildServiceRequest(Request& req) {
                RCLCPP_INFO(node->get_logger(),"Set pen to %d %d %d %d %d",cfg->get<bool>("on"),
                        cfg->get<int>("r"),cfg->get<int>("g"),cfg->get<int>("b"),cfg->get<int>("width"));
                req.off = !cfg->get<bool>("on");
                req.r=cfg->get<int>("r");
                req.g=cfg->get<int>("g");
                req.b=cfg->get<int>("b");
                req.width=cfg->get<int>("width");
            }
        public:
            TaskSetPen(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetPen() {};
    };

    class TaskFactorySetPen : public TaskDefinition<TaskSetPenConfig, TurtleSimEnv, TaskSetPen>
    {
        public:
            TaskFactorySetPen(TaskEnvironmentPtr env) : 
                Parent("SetPen","Set pen value",true,env) {}
            virtual ~TaskFactorySetPen() {};
    };
}

#endif // TASK_SETPEN_H
