#ifndef TASK_SPAWN_H
#define TASK_SPAWN_H

#include "turtlesim/srv/spawn.hpp"
#include "task_manager_lib/TaskServiceGeneric.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    struct TaskSpawnConfig : public TaskServiceGenericConfig {
        TaskSpawnConfig() : TaskServiceGenericConfig("/spawn") {
            define("x",0.0,"x coordinate of the new turtle",true);
            define("y",0.0,"y coordinate of the new turtle",true);
            define("theta",0.0,"heading of the new turtle",true);
            define("name","","name the new turtle",true);
        }
    };

    class TaskSpawn : public TaskServiceGeneric<turtlesim::srv::Spawn,TaskSpawnConfig,TurtleSimEnv>
    {
        protected:

            virtual void buildServiceRequest(Request& req) {
                req.x=cfg->get<double>("x");
                req.y=cfg->get<double>("y");
                req.theta=cfg->get<double>("theta");
                req.name=cfg->get<std::string>("name");
                RCLCPP_INFO(node->get_logger(),"Spawning at %f %f %f %s",
                        req.x,req.y,req.theta,req.name.c_str());
            }

        public:
            TaskSpawn(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSpawn() {};
    };

    class TaskFactorySpawn : public TaskDefinition<TaskSpawnConfig, TurtleSimEnv, TaskSpawn>
    {

        public:
            TaskFactorySpawn(TaskEnvironmentPtr env) : 
                Parent("Spawn","Spawn a new turtle",true,env) {}
            virtual ~TaskFactorySpawn() {};
    };
}

#endif // TASK_SPAWN_H
