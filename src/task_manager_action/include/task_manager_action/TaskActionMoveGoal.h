#ifndef TASK_GOTO_MOVE_GOAL_H
#define TASK_GOTO_MOVE_GOAL_H

#include <task_manager_lib/TaskDefinition.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "task_manager_action/TaskActionMoveGoalConfig.h"

namespace task_manager_action {
    template <class TaskEnvironment>
    class TaskActionMoveGoal : 
        public task_manager_lib::TaskInstance<TaskActionMoveGoalConfig, TaskEnvironment>
    {
        protected:
            typedef task_manager_lib::TaskInstance<TaskActionMoveGoalConfig, TaskEnvironment> Parent;

            ros::Time init_time;
            ros::Publisher goal_pub;
            tf::TransformListener listener;
        public:
            TaskActionMoveGoal(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskActionMoveGoal() {};

            virtual task_manager_lib::TaskIndicator initialise() {
                const TaskActionMoveGoalConfig & cfg = Parent::cfg;
                ros::NodeHandle & nh = Parent::env->getNodeHandle();
                goal_pub = nh.advertise<geometry_msgs::PoseStamped>(cfg.topic_name,1);
                // Let the publisher start
                ros::Duration(0.05).sleep();
                init_time = ros::Time::now();
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = cfg.reference_frame;
                goal.header.stamp = ros::Time::now();

                goal.pose.position.x = cfg.goal_x;
                goal.pose.position.y = cfg.goal_y;
                goal.pose.position.z = cfg.goal_z;
                goal.pose.orientation = 
                    tf::createQuaternionMsgFromRollPitchYaw(cfg.goal_roll,cfg.goal_pitch,cfg.goal_yaw);
                goal_pub.publish(goal);
                ROS_INFO("Published goal to %s",cfg.topic_name.c_str());

                return task_manager_msgs::TaskStatus::TASK_INITIALISED;
            }

            virtual task_manager_lib::TaskIndicator iterate() {
                const TaskActionMoveGoalConfig & cfg = Parent::cfg;
                if (cfg.wait_completion) {
                    if (cfg.resend_goal && ((ros::Time::now() - init_time).toSec() > 1.0)) {
                        geometry_msgs::PoseStamped goal;
                        goal.header.frame_id = cfg.reference_frame;
                        goal.header.stamp = ros::Time::now();

                        goal.pose.position.x = cfg.goal_x;
                        goal.pose.position.y = cfg.goal_y;
                        goal.pose.position.z = cfg.goal_z;
                        goal.pose.orientation = 
                            tf::createQuaternionMsgFromRollPitchYaw(cfg.goal_roll,cfg.goal_pitch,cfg.goal_yaw);
                        goal_pub.publish(goal);
                        init_time = ros::Time::now();
                    }

                    if (!listener.canTransform(cfg.base_link,cfg.reference_frame,ros::Time(0))) {
                        ROS_WARN("Task MoveBase: Cannot estimate current pose");
                        // Waiting for timeout?
                        return task_manager_msgs::TaskStatus::TASK_RUNNING;
                    }
                    tf::StampedTransform state_transform, goal_transform;
                    goal_transform.setOrigin(tf::Vector3(cfg.goal_x,cfg.goal_y,cfg.goal_z));
                    goal_transform.setRotation(tf::createQuaternionFromRPY(cfg.goal_roll,cfg.goal_pitch,cfg.goal_yaw));
                    listener.lookupTransform(cfg.base_link,cfg.reference_frame, ros::Time(0), state_transform);
                    tf::Transform error = goal_transform.inverse() * state_transform;
                    if ((error.getOrigin().length() < cfg.dist_threshold) && 
                            (error.getRotation().getAngle() < cfg.angle_threshold)) {
                        return task_manager_msgs::TaskStatus::TASK_COMPLETED;
                    }
                    return task_manager_msgs::TaskStatus::TASK_RUNNING;
                } else {
                    return task_manager_msgs::TaskStatus::TASK_COMPLETED;
                }
            }
    };

    template <class TaskEnvironment>
    class TaskFactoryActionMoveGoal : public task_manager_lib::TaskDefinition<TaskActionMoveGoalConfig, TaskEnvironment, TaskActionMoveGoal<TaskEnvironment> >
    {
        protected: 
            typedef task_manager_lib::TaskDefinition<TaskActionMoveGoalConfig, TaskEnvironment, TaskActionMoveGoal<TaskEnvironment> > Parent;

        public:
            TaskFactoryActionMoveGoal(task_manager_lib::TaskEnvironmentPtr env) : 
                Parent("ActionMoveGoal","Reach a desired goal using move base",true,env) {}
            virtual ~TaskFactoryActionMoveGoal() {};
    };

};

#endif // TASK_GOTO_MOVE_GOAL_H
