#ifndef TASK_GOTO_MOVE_BASE_H
#define TASK_GOTO_MOVE_BASE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_action/TaskActionGeneric.h"
#include "task_manager_action/TaskActionMoveBaseConfig.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

namespace task_manager_action {
    template <class TaskEnvironment>
    class TaskActionMoveBase : 
        public TaskActionGeneric<move_base_msgs::MoveBaseAction,TaskActionMoveBaseConfig, TaskEnvironment>
    {
        protected:
            typedef TaskActionGeneric<move_base_msgs::MoveBaseAction,
                    TaskActionMoveBaseConfig, TaskEnvironment> Parent;

            const std::string & getActionName() const {
                return Parent::cfg.action_name;
            }
            
            void buildActionGoal(typename Parent::Goal & goal) const {
                const TaskActionMoveBaseConfig & cfg_ = Parent::cfg;
                goal.target_pose.header.frame_id = cfg_.frame_id;
                goal.target_pose.header.stamp = ros::Time::now();

                goal.target_pose.pose.position.x = cfg_.goal_x;
                goal.target_pose.pose.position.y = cfg_.goal_y;
                goal.target_pose.pose.position.z = cfg_.goal_z;
                goal.target_pose.pose.orientation = 
                    tf::createQuaternionMsgFromRollPitchYaw(cfg_.goal_roll,cfg_.goal_pitch,cfg_.goal_yaw);
            }
        public:
            TaskActionMoveBase(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskActionMoveBase() {};
    };

    template <class TaskEnvironment>
    class TaskFactoryActionMoveBase : 
        public task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, TaskEnvironment, TaskActionMoveBase<TaskEnvironment> >
    {
        protected:
            typedef task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, TaskEnvironment, TaskActionMoveBase<TaskEnvironment> > Parent;
        public:
            TaskFactoryActionMoveBase(task_manager_lib::TaskEnvironmentPtr env) : 
                Parent("ActionMoveBase","Publish an action goal for move base",true,env) {}
            virtual ~TaskFactoryActionMoveBase() {};
    };
};

#endif // TASK_GOTO_H
