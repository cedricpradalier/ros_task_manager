#ifndef TURTLE_SIM_ENV_H
#define TURTLE_SIM_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_srvs/Empty.h"
#include "turtlesim/SetPen.h"
#if ROS_VERSION_MINIMUM(1, 10, 0) 
// Hydro
#include "geometry_msgs/Twist.h"
#else
// Groovy and earlier code
#include "turtlesim/Velocity.h"
#endif
#include "turtlesim/Pose.h"
#include "boost/algorithm/string.hpp"
#include "std_msgs/String.h"
#include "task_manager_turtlesim/FollowPathPL.h"

#define TEST_ACTION_CLIENT
#ifdef TEST_ACTION_CLIENT
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/action_definition.h>
#include <move_base_msgs/MoveBaseAction.h>
#endif

namespace task_manager_turtlesim {
    class TurtleSimEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            unsigned int turtleId;
            ros::Subscriber buttonsSub;
            ros::Subscriber poseSub;
            ros::Publisher velPub;
            ros::ServiceClient setPenClt;
            ros::ServiceClient clearClt;
            bool paused;

            void buttonCallback(const std_msgs::String::ConstPtr& msg) {
                if (boost::algorithm::to_lower_copy(msg->data) == "pause") {
                    paused = !paused;
                    if (paused) {
                        ROS_INFO("Mission paused");
                    } else {
                        ROS_INFO("Mission resumed");
                    }
                }
            }

            void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
                tpose = *msg;
            }
            turtlesim::Pose tpose;
#ifdef TEST_ACTION_CLIENT
            ACTION_DEFINITION(move_base_msgs::MoveBaseAction)
            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
            typedef boost::shared_ptr<Client> ClientPtr;
            ClientPtr move_base_action_client;
#endif

            FollowPathPL waypoints;

        public:
            TurtleSimEnv(ros::NodeHandle & nh, unsigned int id=1);
            ~TurtleSimEnv() {};

            const turtlesim::Pose & getPose() const {return tpose;}

            void publishVelocity(double linear, double angular) {
#if ROS_VERSION_MINIMUM(1, 10, 0) 
                geometry_msgs::Twist cmd;
                if (paused) {
                    cmd.linear.x = 0.;
                    cmd.angular.z = 0.;
                } else {
                    cmd.linear.x = linear;
                    cmd.angular.z = angular;
                }
#else
                turtlesim::Velocity cmd;
                if (paused) {
                    cmd.linear = 0.;
                    cmd.angular = 0.;
                } else {
                    cmd.linear = linear;
                    cmd.angular = angular;
                }
#endif
                velPub.publish(cmd);
            }

            void setPen(bool on, unsigned int r=0xFF, unsigned int g=0xFF, unsigned int b=0xFF, unsigned int width=1);

            const FollowPathPL & getWP() const {
                return waypoints;
            }

            FollowPathPL & getWP() {
                return waypoints;
            }

            void popWP() {
                waypoints.pop_front();
            }

            void clear();
#ifdef TEST_ACTION_CLIENT
            ClientPtr getMoveBaseActionClient();
#endif
    };

    typedef boost::shared_ptr<TurtleSimEnv> TurtleSimEnvPtr;
    typedef boost::shared_ptr<TurtleSimEnv const> TurtleSimEnvConstPtr;
};

#endif // TURTLE_SIM_ENV_H
