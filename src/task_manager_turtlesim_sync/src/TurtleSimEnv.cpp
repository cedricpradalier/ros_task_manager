
#include "task_manager_turtlesim_sync/TurtleSimEnv.h"

using namespace task_manager_turtlesim_sync;

TurtleSimEnv::TurtleSimEnv(ros::NodeHandle & n, 
        const std::string & name, unsigned int id) : task_manager_sync::TaskEnvironmentSync(n,name,"sync"),
    turtleId(id)
{
    char buffer[128]; sprintf(buffer,"/turtle%d",id);
    std::string tname(buffer);

    poseSub = nh.subscribe(tname+"/pose",1,&TurtleSimEnv::poseCallback,this);
#if ROS_VERSION_MINIMUM(1, 10, 0) 
    velPub = nh.advertise<geometry_msgs::Twist>(tname+"/cmd_vel",1);
#else
    velPub = nh.advertise<turtlesim::Velocity>(tname+"/command_velocity",1);
#endif
}

