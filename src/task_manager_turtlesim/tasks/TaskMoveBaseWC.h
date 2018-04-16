#ifndef TASK_MOVE_BASE_WC_H
#define TASK_MOVE_BASE_WC_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_action/TaskActionMoveBase.h"

using namespace task_manager_lib;
using namespace task_manager_action;

// There is no move_base for turtlesim, this is to test the principle of the
// generic TaskActionMoveBase
namespace task_manager_turtlesim {
    class TaskMoveBaseWC : public TaskActionGenericWithoutClient<move_base_msgs::MoveBaseAction,TaskActionMoveBaseConfig, TurtleSimEnv>
    {
        protected:
            typedef TaskActionGenericWithoutClient<move_base_msgs::MoveBaseAction,
                    TaskActionMoveBaseConfig, TurtleSimEnv> Parent;

            virtual typename Parent::ClientPtr getActionClient() {
                return env->getMoveBaseActionClient();
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
            TaskMoveBaseWC(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                Parent(def,env) {}
            virtual ~TaskMoveBaseWC() {};
    };

    class TaskFactoryMoveBaseWC : public task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, TurtleSimEnv, TaskMoveBaseWC>
    {
        protected:
            typedef task_manager_lib::TaskDefinition<TaskActionMoveBaseConfig, TurtleSimEnv, TaskMoveBaseWC> Parent;
        public:
            TaskFactoryMoveBaseWC(TaskEnvironmentPtr env) :
                Parent("ActionMoveBaseWC","Publish an action goal for move base, using client from the environment",true,env) {}
            virtual ~TaskFactoryMoveBaseWC() {};
    };
};

#endif // TASK_MOVE_BASE_WC_H
