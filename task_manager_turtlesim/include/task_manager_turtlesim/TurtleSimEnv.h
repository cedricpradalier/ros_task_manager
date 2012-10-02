#ifndef TURTLE_SIM_ENV_H
#define TURTLE_SIM_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_srvs/Empty.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Velocity.h"
#include "turtlesim/Pose.h"

namespace task_manager_turtlesim {
    class TurtleSimEnv: public TaskEnvironment
    {
        protected:
            unsigned int turtleId;
            ros::NodeHandle nh;
            ros::Subscriber poseSub;
            ros::Publisher velPub;
            ros::ServiceClient setPenClt;
            ros::ServiceClient clearClt;

            void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
                tpose = *msg;
            }
            turtlesim::Pose tpose;

        public:
            TurtleSimEnv(ros::NodeHandle & nh, unsigned int id=1);
            ~TurtleSimEnv() {};

            const turtlesim::Pose & getPose() const {return tpose;}

            void publishVelocity(double linear, double angular) {
                turtlesim::Velocity cmd;
                cmd.linear = linear;
                cmd.angular = angular;
                velPub.publish(cmd);
            }

            void setPen(bool on, unsigned int r=0xFF, unsigned int g=0xFF, unsigned int b=0xFF, unsigned int width=1);

            void clear();
    };

};

#endif // TURTLE_SIM_ENV_H
