#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_sync/TaskServerSync.h"

#include "task_manager_turtlesim_sync/TurtleSimEnv.h"


using namespace task_manager_turtlesim_sync;
using namespace task_manager_lib;

class TaskServer : public task_manager_sync::TaskServerSync {
    public:
        TaskServer(TaskEnvironmentPtr _env) : task_manager_sync::TaskServerSync(_env) {
            start();
        }

};



int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);//init ros
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtlesim_tasks");
    std::string partner_name = "partner";
    int id = 1;
    node->declare_parameter("my_name", partner_name);
    node->declare_parameter("my_id", id);
    partner_name = node->get_parameter("my_name").get_parameter_value().get<std::string>();
    id = node->get_parameter("my_id").get_parameter_value().get<int>();

    TurtleSimEnvPtr env(new TurtleSimEnv(node,partner_name,id));
    env->addSyncSource("partner1");
    env->addSyncSource("partner2");
    TaskServer ts(env);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
