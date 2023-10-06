#ifndef TASK_nav2_H
#define TASK_nav2_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_nav2/TaskActionNav2.h"

using namespace task_manager_lib;
using namespace task_manager_nav2;

// There is no nav2 for turtlesim, this is to test the principle of the
// generic TaskActionNav2
namespace task_manager_turtlesim {
    class TaskNav2 : public TaskActionNav2<TurtleSimEnv>
    {
            TaskNav2(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionNav2<TurtleSimEnv>(def,env) {}
            virtual ~TaskNav2() {};
    };

    class TaskFactoryNav2 : public TaskFactoryActionNav2<TurtleSimEnv>
    {

        public:
            TaskFactoryNav2(TaskEnvironmentPtr env) :
                TaskFactoryActionNav2<TurtleSimEnv>("ActionNav2",env) {}
            virtual ~TaskFactoryNav2() {};
    };
}

#endif // TASK_nav2_H
