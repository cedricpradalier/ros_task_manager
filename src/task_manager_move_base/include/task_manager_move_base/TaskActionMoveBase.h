#ifndef TASK_GOTO_MOVE_BASE_H
#define TASK_GOTO_MOVE_BASE_H

#include "task_manager_lib/TaskActionGeneric.h"
#include <move_base_msgs/action/move_base.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace task_manager_move_base {
    struct TaskActionMoveBaseConfig : public task_manager_lib::TaskActionGenericConfig {
        TaskActionMoveBaseConfig() : task_manager_lib::TaskActionGenericConfig("/move_base") {
            define("goal_x",        0.0, "X coordinate of destination (m)",true);
            define("goal_y",        0.0, "Y coordinate of destination (m)",true);
            define("goal_z",        0.0, "Z coordinate of destination (m)",true);
            define("goal_roll",     0.0, "Roll at destination (rad)",true);
            define("goal_pitch",    0.0, "Pitch at destination (rad)",true);
            define("goal_yaw",      0.0, "Yaw/Heading at destination (rad)",true);
            define("frame_id",      "world", "Name of the reference frame",true);
        }

    };

    template <class Environment>
        class TaskActionMoveBase : 
            public task_manager_lib::TaskActionGeneric<move_base_msgs::action::MoveBase,TaskActionMoveBaseConfig, Environment>
    {
        protected:
            typedef task_manager_lib::TaskActionGeneric<move_base_msgs::action::MoveBase,
                    TaskActionMoveBaseConfig, Environment> Parent;

            void buildActionGoal(typename Parent::Goal & goal) {
                std::shared_ptr<const TaskActionMoveBaseConfig> cfg_ = 
                    std::dynamic_pointer_cast<const TaskActionMoveBaseConfig,const task_manager_lib::TaskConfig>(
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
                goal.target_pose.header = TMstamped.header;
                goal.target_pose.pose.position.x = TMstamped.transform.translation.x;
                goal.target_pose.pose.position.y = TMstamped.transform.translation.y;
                goal.target_pose.pose.position.z = TMstamped.transform.translation.z;
                goal.target_pose.pose.orientation = TMstamped.transform.rotation;

            }
        public:
            TaskActionMoveBase(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskActionMoveBase() {};
    };

    template <class Environment>
        class TaskFactoryActionMoveBase : 
            public task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, Environment, TaskActionMoveBase<Environment> >
    {
        protected:
            typedef task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, Environment, TaskActionMoveBase<Environment> > Parent;
        public:
            TaskFactoryActionMoveBase(task_manager_lib::TaskEnvironmentPtr env) : 
                Parent("ActionMoveBase","Publish an action goal for move base",true,env) {}
            TaskFactoryActionMoveBase(const std::string & name, task_manager_lib::TaskEnvironmentPtr env) : 
                Parent(name,"Publish an action goal for move base",true,env) {}
            virtual ~TaskFactoryActionMoveBase() {};
    };
}

#endif // TASK_GOTO_H
