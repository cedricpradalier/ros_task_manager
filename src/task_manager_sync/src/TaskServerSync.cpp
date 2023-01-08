#include <rclcpp/rclcpp.hpp>
#include <task_manager_sync/TaskServerSync.h>


using namespace task_manager_sync ;
TaskServerSync::TaskServerSync(TaskEnvironmentPtr _env, bool default_wait) : TaskServerBase(_env,default_wait) {
    task_manager_sync::TaskEnvironmentSyncPtr senv = 
        std::dynamic_pointer_cast<task_manager_sync::TaskEnvironmentSync>(_env);
    if (!senv) {
        RCLCPP_ERROR(node->get_logger(),"TaskServerSync initialized with an environment not inherited from TaskEnvironmentSync");
        assert(senv);
    }
    ts.addTask(task_manager_lib::TaskDefinitionPtr(
                new task_manager_sync::TaskFactorySetStatusSync(senv)));
    ts.addTask(task_manager_lib::TaskDefinitionPtr(
                new task_manager_sync::TaskFactoryWaitForStatusSync(senv)));
}

