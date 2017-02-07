#ifndef TASK_MOVE_BASE_H
#define TASK_MOVE_BASE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_action/TaskActionMoveBase.h"

using namespace task_manager_lib;
using namespace task_manager_action;

// There is no move_base for turtlesim, this is to test the principle of the
// generic TaskActionMoveBase
namespace task_manager_turtlesim {
    class TaskMoveBase : public TaskActionMoveBase<TurtleSimEnv>
    {
            TaskMoveBase(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveBase<TurtleSimEnv>(def,env) {}
            virtual ~TaskMoveBase() {};
    };

    class TaskFactoryMoveBase : public TaskFactoryActionMoveBase<TurtleSimEnv>
    {

        public:
            TaskFactoryMoveBase(TaskEnvironmentPtr env) :
                TaskFactoryActionMoveBase<TurtleSimEnv>(env) {}
            virtual ~TaskFactoryMoveBase() {};
    };
};

#endif // TASK_MOVE_BASE_H
