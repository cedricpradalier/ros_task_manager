#include <math.h>
#include "TaskSetPen.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskIndicator TaskSetPen::initialise()
{
    // RCLCPP_INFO(node->get_logger(),"SetPen Task Config");
    // cfg->printConfig();
    
    state = WAITING_FOR_CLIENT;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskSetPen::iterate()
{
    switch (state) {
        case WAITING_FOR_CLIENT:
            if (!env->isSetPenAvailable()) {
                RCLCPP_INFO(node->get_logger(),"TaskSetPen: Waiting for Service");
                break;
            }
            RCLCPP_INFO(node->get_logger(),"Set pen to %d %d %d %d %d",cfg->get<bool>("on"),
                    cfg->get<int>("r"),cfg->get<int>("g"),cfg->get<int>("b"),cfg->get<int>("width"));
            future = env->setPenAsync(cfg->get<bool>("on"),
                    cfg->get<int>("r"),cfg->get<int>("g"),cfg->get<int>("b"),cfg->get<int>("width"));
            state = WAITING_FOR_FUTURE;
            // fallthrough
        case WAITING_FOR_FUTURE:
            if (!future.valid()) {
                RCLCPP_ERROR(node->get_logger(),"TaskSetPen: No Future");
                return TaskStatus::TASK_FAILED;
            }
            RCLCPP_INFO(node->get_logger(),"TaskSetPen: Waiting for future");
            if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                RCLCPP_INFO(node->get_logger(),"TaskSetPen: received future");
                return TaskStatus::TASK_COMPLETED;
            }
            RCLCPP_INFO(node->get_logger(),"TaskSetPen: Running");
            break;
    }
    return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactorySetPen)
