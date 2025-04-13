#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskServerDefault.h"

#include "task_manager_turtlesim/TurtleSimEnv.h"


using namespace task_manager_turtlesim;
using namespace task_manager_lib;

class TaskServer : public TaskServerBase {
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
            addSystemTask();
            start();
        }

};



int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);//init ros
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtlesim_tasks");
    TaskEnvironmentPtr env(new TurtleSimEnv(node,1));
    TaskServer ts(env);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
