#ifndef TASK_GOTO_NAV2_H
#define TASK_GOTO_NAV2_NAV2_H

#include "task_manager_lib/TaskActionGeneric.h"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace task_manager_nav2 {
    struct TaskActionNav2Config : public task_manager_lib::TaskActionGenericConfig {
        TaskActionNav2Config() : task_manager_lib::TaskActionGenericConfig("/nav2") {
            define("goal_x",        0.0, "X coordinate of destination (m)",true);
            define("goal_y",        0.0, "Y coordinate of destination (m)",true);
            define("goal_z",        0.0, "Z coordinate of destination (m)",true);
            define("goal_roll",     0.0, "Roll at destination (rad)",true);
            define("goal_pitch",    0.0, "Pitch at destination (rad)",true);
            define("goal_yaw",      0.0, "Yaw/Heading at destination (rad)",true);
            define("behaviour",      "MainTree", "Name of the behaviour tree",true);
            define("frame_id",      "world", "Name of the reference frame",true);
        }

    };

    template <class Environment>
        class TaskActionNav2 : 
            public task_manager_lib::TaskActionGeneric<nav2_msgs::action::NavigateToPose,TaskActionNav2Config, Environment>
    {
        protected:
            typedef task_manager_lib::TaskActionGeneric<nav2_msgs::action::NavigateToPose,
                    TaskActionNav2Config, Environment> Parent;

            void buildActionGoal(typename Parent::Goal & goal) {
                std::shared_ptr<const TaskActionNav2Config> cfg_ = 
                    std::dynamic_pointer_cast<const TaskActionNav2Config,const task_manager_lib::TaskConfig>(
                            task_manager_lib::TaskInstanceBase::getConfig());
                assert(cfg_);
                tf2::Transform T;
                T.setOrigin(tf2::Vector3(cfg_->get<double>("goal_x"),
                            cfg_->get<double>("goal_y"),
                            cfg_->get<double>("goal_z")));
                tf2::Quaternion Q;
                Q.setRPY(cfg_->get<double>("goal_roll"),
                        cfg_->get<double>("goal_pitch"),
                        cfg_->get<double>("goal_yaw"));
                T.setRotation(Q);
                rclcpp::Time ros_now = this->getNode()->get_clock()->now();
                tf2::TimePoint now(std::chrono::nanoseconds(ros_now.nanoseconds()));
                tf2::Stamped<tf2::Transform> Tstamped(T, now, cfg_->get<std::string>("frame_id"));
                geometry_msgs::msg::TransformStamped TMstamped = tf2::toMsg(Tstamped);
                goal.pose.header = TMstamped.header;
                goal.pose.pose.position.x = TMstamped.transform.translation.x;
                goal.pose.pose.position.y = TMstamped.transform.translation.y;
                goal.pose.pose.position.z = TMstamped.transform.translation.z;
                goal.pose.pose.orientation = TMstamped.transform.rotation;
                goal.behavior_tree = cfg_->get<std::string>("behaviour");

            }
        public:
            TaskActionNav2(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskActionNav2() {};
    };

    template <class Environment>
        class TaskFactoryActionNav2 : 
            public task_manager_lib::TaskDefinition<TaskActionNav2Config, Environment, TaskActionNav2<Environment> >
    {
        protected:
            typedef task_manager_lib::TaskDefinition<TaskActionNav2Config, Environment, TaskActionNav2<Environment> > Parent;
        public:
            TaskFactoryActionNav2(task_manager_lib::TaskEnvironmentPtr env) : 
                Parent("ActionNav2","Publish an action goal for nav2",true,env) {}
            TaskFactoryActionNav2(const std::string & name, task_manager_lib::TaskEnvironmentPtr env) : 
                Parent(name,"Publish an action goal for nav2",true,env) {}
            virtual ~TaskFactoryActionNav2() {};
    };
}

#endif // TASK_GOTO_H
