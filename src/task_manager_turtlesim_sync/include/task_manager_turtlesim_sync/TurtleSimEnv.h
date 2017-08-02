#ifndef TURTLE_SIM_ENV_H
#define TURTLE_SIM_ENV_H

#include <ros/ros.h>
#include "task_manager_sync/TaskEnvironmentSync.h"
#include "std_srvs/Empty.h"
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

namespace task_manager_turtlesim_sync {
    class TurtleSimEnv: public task_manager_sync::TaskEnvironmentSync
    {
        protected:
            unsigned int turtleId;
            ros::Subscriber poseSub;
            ros::Publisher velPub;

            void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
                tpose = *msg;
            }
            turtlesim::Pose tpose;

        public:
            TurtleSimEnv(ros::NodeHandle & nh, const std::string & name, unsigned int id=1);
            ~TurtleSimEnv() {};

            const turtlesim::Pose & getPose() const {return tpose;}

            void publishVelocity(double linear, double angular) {
#if ROS_VERSION_MINIMUM(1, 10, 0) 
                geometry_msgs::Twist cmd;
                cmd.linear.x = linear;
                cmd.angular.z = angular;
#else
                turtlesim::Velocity cmd;
                cmd.linear = linear;
                cmd.angular = angular;
#endif
                velPub.publish(cmd);
            }
    };

    typedef boost::shared_ptr<TurtleSimEnv> TurtleSimEnvPtr;
    typedef boost::shared_ptr<TurtleSimEnv const> TurtleSimEnvConstPtr;
};

#endif // TURTLE_SIM_ENV_H
