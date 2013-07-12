#ifndef TASK_SETPEN_H
#define TASK_SETPEN_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_turtlesim/TurtleSimEnv.h"
#include "task_manager_turtlesim/TaskSetPenConfig.h"
using namespace task_manager_lib;

namespace task_manager_turtlesim {
    class TaskSetPen : public TaskInstance<TaskSetPenConfig,TurtleSimEnv>
    {
        public:
            TaskSetPen(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetPen() {};

            virtual TaskIndicator iterate();

    };
    class TaskFactorySetPen : public TaskDefinition<TaskSetPenConfig, TurtleSimEnv, TaskSetPen>
    {
        public:
            TaskFactorySetPen(TaskEnvironmentPtr env) : 
                Parent("SetPen","Set pen value",false,env) {}
            virtual ~TaskFactorySetPen() {};
    };
};

#endif // TASK_SETPEN_H
