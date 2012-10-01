
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_turtlesim;

TurtleSimEnv::TurtleSimEnv(ros::NodeHandle & n, unsigned int id) :
    turtleId(id), nh(n) 
{
    char buffer[128]; sprintf(buffer,"/turtle%d",id);
    std::string tname(buffer);
    setPenClt = nh.serviceClient<turtlesim::SetPen>(tname+"/set_pen");

    poseSub = nh.subscribe(tname+"/pose",1,&TurtleSimEnv::poseCallback,this);
    velPub = nh.advertise<turtlesim::Velocity>(tname+"/command_velocity",1);
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

