#include "TaskClear.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskClear::initialise()
{
    state = WAITING_FOR_CLIENT;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskClear::iterate()
{
    switch (state) {
        case WAITING_FOR_CLIENT:
            if (!env->isClearAvailable()) {
                RCLCPP_INFO(node->get_logger(),"TaskClear: Waiting for Service");
                break;
            }
            future = env->clearAsync();
            state = WAITING_FOR_FUTURE;
            // fallthrough
        case WAITING_FOR_FUTURE:
            if (!future.valid()) {
                RCLCPP_ERROR(node->get_logger(),"TaskClear: No Future");
                return TaskStatus::TASK_FAILED;
            }
            RCLCPP_INFO(node->get_logger(),"TaskClear: Waiting for future");
            if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                RCLCPP_INFO(node->get_logger(),"TaskClear: received future");
                return TaskStatus::TASK_COMPLETED;
            }
            RCLCPP_INFO(node->get_logger(),"TaskClear: Running");
            break;
    }
    return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryClear)
