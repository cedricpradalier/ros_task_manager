#ifndef TURTLE_SIM_ENV_H
#define TURTLE_SIM_ENV_H

#include <rclcpp/rclcpp.hpp>
#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_sync/TaskEnvironmentSync.h"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


namespace task_manager_turtlesim_sync {
    class TurtleSimEnv: public task_manager_sync::TaskEnvironmentSync
    {
        protected:
            unsigned int turtleId;
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;


            turtlesim::msg::Pose tpose;
            void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
                tpose = *msg;
            }

        public:
            TurtleSimEnv(std::shared_ptr<rclcpp::Node> node, const std::string & name, unsigned int id=1);
            ~TurtleSimEnv() {};

            DECLARE_ENV_CHECKSUM;

            const turtlesim::msg::Pose & getPose() const {return tpose;}

            void publishVelocity(double linear, double angular) {
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = linear;
                cmd.angular.z = angular;
                velPub->publish(cmd);
            }
    };

    typedef std::shared_ptr<TurtleSimEnv> TurtleSimEnvPtr;
    typedef std::shared_ptr<TurtleSimEnv const> TurtleSimEnvConstPtr;
}

#endif // TURTLE_SIM_ENV_H
