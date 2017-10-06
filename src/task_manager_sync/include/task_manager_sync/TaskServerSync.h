#ifndef TASK_SERVER_SYNC_H
#define TASK_SERVER_SYNC_H

#include <ros/ros.h>
#include <task_manager_lib/TaskDefinition.h>
#include <task_manager_lib/TaskServerDefault.h>
#include <task_manager_sync/TaskSetStatusSync.h>
#include <task_manager_sync/TaskWaitForStatusSync.h>


namespace task_manager_sync {
    class TaskServerSync : public task_manager_lib::TaskServerBase {
        public:
            TaskServerSync(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
                task_manager_sync::TaskEnvironmentSyncPtr senv = 
                    boost::dynamic_pointer_cast<task_manager_sync::TaskEnvironmentSync>(_env);
                if (!senv) {
                    ROS_ERROR("TaskServerSync initialized with an environment not inherited from TaskEnvironmentSync");
                    assert(senv);
                }
                ts.addTask(task_manager_lib::TaskDefinitionPtr(
                            new task_manager_sync::TaskFactorySetStatusSync(senv)));
                ts.addTask(task_manager_lib::TaskDefinitionPtr(
                            new task_manager_sync::TaskFactoryWaitForStatusSync(senv)));
            }
    };

};

#endif // TASK_SERVER_SYNC_H

