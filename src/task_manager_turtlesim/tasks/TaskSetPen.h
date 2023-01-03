#ifndef TASK_SETPEN_H
#define TASK_SETPEN_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskSetPenConfig : public TaskConfig {
        TaskSetPenConfig() {
            define("r",128,"Red component",true); 
            define("g",128,"Green component",true); 
            define("b",128,"Blue component",true); 
            define("width",1,"Line width",true); 
            define("on",true,"Activate pen",true); 
        }
    };

    class TaskSetPen : public TaskInstance<TaskSetPenConfig,TurtleSimEnv>
    {
        protected:
            typedef enum {
                WAITING_FOR_CLIENT,
                WAITING_FOR_FUTURE
            } ClientState;
            ClientState state;
            rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future;
        public:
            TaskSetPen(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetPen() {};

            virtual TaskIndicator initialise();
            virtual TaskIndicator iterate();

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
