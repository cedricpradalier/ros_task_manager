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

            void clear();
    };

    typedef boost::shared_ptr<TurtleSimEnv> TurtleSimEnvPtr;
    typedef boost::shared_ptr<TurtleSimEnv const> TurtleSimEnvConstPtr;
};

#endif // TURTLE_SIM_ENV_H
