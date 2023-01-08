
#include "task_manager_turtlesim_sync/TurtleSimEnv.h"

using namespace task_manager_turtlesim_sync;

TurtleSimEnv::TurtleSimEnv(std::shared_ptr<rclcpp::Node> n, const std::string & name, unsigned int id) : task_manager_sync::TaskEnvironmentSync(n,name),
    turtleId(id)
{
    char buffer[128]; sprintf(buffer,"/turtle%d",id);
    std::string tname(buffer);
    poseSub = node->create_subscription<turtlesim::msg::Pose>(tname+"/pose",1,std::bind(&TurtleSimEnv::poseCallback,this,std::placeholders::_1));
    velPub = node->create_publisher<geometry_msgs::msg::Twist>(tname+"/cmd_vel",1);
}

