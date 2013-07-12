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
            start();
        }

};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"turtlesim_tasks");//init ros
    ros::NodeHandle nh("~");
    TaskEnvironmentPtr env(new TurtleSimEnv(nh,1));
    TaskServer ts(env);
    ros::spin();
    return 0;
}
