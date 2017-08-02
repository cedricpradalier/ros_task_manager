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
    ros::init(argc,argv,"turtlesim_tasks");//init ros
    ros::NodeHandle nh("~");
    std::string partner_name = "partner";
    int id = 1;
    nh.getParam("my_name",partner_name);
    nh.getParam("my_id",id);

    TurtleSimEnvPtr env(new TurtleSimEnv(nh,partner_name,id));
    env->addSyncSource("partner1");
    env->addSyncSource("partner2");
    TaskServer ts(env);
    ros::spin();
    return 0;
}
