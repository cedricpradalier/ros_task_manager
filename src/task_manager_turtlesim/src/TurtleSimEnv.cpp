
#include "task_manager_turtlesim/TurtleSimEnv.h"

using namespace task_manager_turtlesim;

TurtleSimEnv::TurtleSimEnv(std::shared_ptr<rclcpp::Node> n, unsigned int id) : task_manager_lib::TaskEnvironment(n),
    turtleId(id), paused(false)
{
    char buffer[128]; sprintf(buffer,"/turtle%d",id);
    std::string tname(buffer);
    clearClt = node->create_client<std_srvs::srv::Empty>("/clear");
    setPenClt = node->create_client<turtlesim::srv::SetPen>(tname+"/set_pen");

    buttonsSub = node->create_subscription<std_msgs::msg::String>("/buttons",1,std::bind(&TurtleSimEnv::buttonCallback,this,std::placeholders::_1));
    poseSub = node->create_subscription<turtlesim::msg::Pose>(tname+"/pose",1,std::bind(&TurtleSimEnv::poseCallback,this,std::placeholders::_1));
    velPub = node->create_publisher<geometry_msgs::msg::Twist>(tname+"/cmd_vel",1);
}

bool TurtleSimEnv::isSetPenAvailable() {
    return setPenClt->service_is_ready();
}
   
rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture TurtleSimEnv::setPenAsync(bool on, unsigned int r, unsigned int g, unsigned int b, unsigned int width)
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->off = !on;
    request->width = width;

    return setPenClt->async_send_request(request).future.share();
}

bool TurtleSimEnv::isClearAvailable() {
    return clearClt->service_is_ready();
}

rclcpp::Client<std_srvs::srv::Empty>::SharedFuture TurtleSimEnv::clearAsync()
{
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    return clearClt->async_send_request(request).future.share();
}

#ifdef TEST_ACTION_CLIENT
TurtleSimEnv::ClientPtr TurtleSimEnv::getMoveBaseActionClient() {
    if (!move_base_action_client) {
        move_base_action_client = 
            rclcpp_action::create_client<move_base_msgs::action::MoveBase>( node, "/move_base");
        if (!move_base_action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(),"Action server /move_base did not reply after 5s, aborting task");
            move_base_action_client.reset();
        }
    }
    return move_base_action_client;
}
#endif
