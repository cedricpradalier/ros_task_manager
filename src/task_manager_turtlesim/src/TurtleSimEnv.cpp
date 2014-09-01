
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_turtlesim;

TurtleSimEnv::TurtleSimEnv(ros::NodeHandle & n, unsigned int id) : task_manager_lib::TaskEnvironment(n),
    turtleId(id), paused(false)
{
    char buffer[128]; sprintf(buffer,"/turtle%d",id);
    std::string tname(buffer);
    clearClt = nh.serviceClient<std_srvs::Empty>("/clear");
    setPenClt = nh.serviceClient<turtlesim::SetPen>(tname+"/set_pen");

    buttonsSub = nh.subscribe("/buttons",10,&TurtleSimEnv::buttonCallback,this);
    poseSub = nh.subscribe(tname+"/pose",1,&TurtleSimEnv::poseCallback,this);
#if ROS_VERSION_MINIMUM(1, 10, 0) 
    velPub = nh.advertise<geometry_msgs::Twist>(tname+"/cmd_vel",1);
#else
    velPub = nh.advertise<turtlesim::Velocity>(tname+"/command_velocity",1);
#endif
}

void TurtleSimEnv::setPen(bool on, unsigned int r, unsigned int g, unsigned int b, unsigned int width)
{
    turtlesim::SetPen setpen;
    setpen.request.r = r;
    setpen.request.g = g;
    setpen.request.b = b;
    setpen.request.off = !on;
    setpen.request.width = width;
    if (!setPenClt.call(setpen)) {
        ROS_ERROR("Failed to call service set_pen");
    }
}

void TurtleSimEnv::clear()
{
    std_srvs::Empty nothing;
    if (!clearClt.call(nothing)) {
        ROS_ERROR("Failed to call service clear");
    }
}

