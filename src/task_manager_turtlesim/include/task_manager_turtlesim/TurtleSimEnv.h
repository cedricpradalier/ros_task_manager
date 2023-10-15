#ifndef TURTLE_SIM_ENV_H
#define TURTLE_SIM_ENV_H

#include <rclcpp/rclcpp.hpp>
#include "task_manager_lib/TaskDefinition.h"
#include "std_srvs//srv/empty.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "boost/algorithm/string.hpp"
#include "std_msgs/msg/string.hpp"

// #define TEST_ACTION_CLIENT
#ifdef TEST_ACTION_CLIENT
#include <move_base_msgs/action/move_base.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#endif

namespace task_manager_turtlesim {
    class TurtleSimEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            unsigned int turtleId;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr buttonsSub;
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
            rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr setPenClt;
            rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clearClt;
            bool paused;

            void buttonCallback(const std_msgs::msg::String::SharedPtr msg) {
                if (boost::algorithm::to_lower_copy(msg->data) == "pause") {
                    paused = !paused;
                    if (paused) {
                        RCLCPP_INFO(this->getNode()->get_logger(), "Mission paused");
                    } else {
                        RCLCPP_INFO(this->getNode()->get_logger(), "Mission resumed");
                    }
                }
            }

            turtlesim::msg::Pose tpose;
            void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
                tpose = *msg;
            }

#ifdef TEST_ACTION_CLIENT
            typedef rclcpp_action::Client<move_base_msgs::action::MoveBase>::SharedPtr ClientPtr;
            ClientPtr move_base_action_client;
#endif


        public:
            TurtleSimEnv(std::shared_ptr<rclcpp::Node> node, unsigned int id=1);
            ~TurtleSimEnv() {};

            DECLARE_ENV_CHECKSUM;

            const turtlesim::msg::Pose & getPose() const {return tpose;}

            void publishVelocity(double linear, double angular) {
                geometry_msgs::msg::Twist cmd;
                if (paused) {
                    cmd.linear.x = 0.;
                    cmd.angular.z = 0.;
                } else {
                    cmd.linear.x = linear;
                    cmd.angular.z = angular;
                }
                velPub->publish(cmd);
            }

            bool isSetPenAvailable();
            rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture setPenAsync(bool on, unsigned int r=0xFF, unsigned int g=0xFF, unsigned int b=0xFF, unsigned int width=1);
            rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr getSetPenClient() {return setPenClt;}

            bool isClearAvailable();
            rclcpp::Client<std_srvs::srv::Empty>::SharedFuture clearAsync();
            rclcpp::Client<std_srvs::srv::Empty>::SharedPtr getClearClient() {return clearClt;}


#ifdef TEST_ACTION_CLIENT
            ClientPtr getMoveBaseActionClient();
#endif
    };

    typedef std::shared_ptr<TurtleSimEnv> TurtleSimEnvPtr;
    typedef std::shared_ptr<TurtleSimEnv const> TurtleSimEnvConstPtr;
}

#endif // TURTLE_SIM_ENV_H
