#ifndef TASK_MOVE_GOAL_H
#define TASK_MOVE_GOAL_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_action/TaskActionMoveGoal.h"

using namespace task_manager_lib;
using namespace task_manager_action;

// There is no move_goal for turtlesim, this is to test the principle of the
// generic TaskActionMoveGoal
namespace task_manager_turtlesim {
    class TaskMoveGoal : public TaskActionMoveGoal<TurtleSimEnv>
    {
            TaskMoveGoal(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveGoal<TurtleSimEnv>(def,env) {}
            virtual ~TaskMoveGoal() {};
    };

    class TaskFactoryMoveGoal : public TaskFactoryActionMoveGoal<TurtleSimEnv>
    {

        public:
            TaskFactoryMoveGoal(TaskEnvironmentPtr env) :
                TaskFactoryActionMoveGoal<TurtleSimEnv>(env) {}
            virtual ~TaskFactoryMoveGoal() {};
    };
};

#endif // TASK_MOVE_GOAL_H
