#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskServerDefault.h"

#include "task_manager_sync/TaskEnvironmentSync.h"


using namespace task_manager_sync;
using namespace task_manager_lib;

class TaskServer : public TaskServerBase {
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
            start();
        }

};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task_server_sync");//init ros
    ros::NodeHandle nh("~");
    std::string partner_name = "partner";
    nh.getParam("my_name",partner_name);

    TaskEnvironmentSyncPtr env(new TaskEnvironmentSync(nh,partner_name,"sync"));
    env->addSyncSource("partner1");
    env->addSyncSource("partner2");
    TaskServer ts(env);
    ros::spin();
    return 0;
}
