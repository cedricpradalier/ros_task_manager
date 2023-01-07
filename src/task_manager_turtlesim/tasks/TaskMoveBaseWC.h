#ifndef TASK_MOVE_BASE_WC_H
#define TASK_MOVE_BASE_WC_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_move_base/TaskActionMoveBase.h"

using namespace task_manager_lib;
using namespace task_manager_move_base;

namespace task_manager_turtlesim {
    class TaskMoveBaseWC : public TaskActionMoveBase<TurtleSimEnv>
    {
        public:
            TaskMoveBaseWC(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveBase<TurtleSimEnv>(def,env) {}
            virtual ~TaskMoveBaseWC() {};

        protected:
            // We need to replace the action client.
            virtual typename Parent::ClientPtr getActionClient() {
                return env->getMoveBaseActionClient();
            }
    };

    class TaskFactoryMoveBaseWC : public TaskFactoryActionMoveBase<TurtleSimEnv>
    {

        public:
            TaskFactoryMoveBaseWC(TaskEnvironmentPtr env) :
                TaskFactoryActionMoveBase<TurtleSimEnv>("ActionMoveBaseWC",env) {}
            virtual ~TaskFactoryMoveBaseWC() {};
    };
}

#endif // TASK_MOVE_BASE_WC_H
