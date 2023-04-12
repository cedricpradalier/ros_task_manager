#include <math.h>
#include "TaskWaitForROI.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

void TaskWaitForROIConfig::update() {
    roi_x = get<double>("roi_x");
    roi_y = get<double>("roi_y");
    roi_radius = get<double>("roi_radius");
}


TaskIndicator TaskWaitForROI::iterate()
{
    cfg->update();
    const turtlesim::msg::Pose & tpose = env->getPose();
    double r = hypot(cfg->roi_y-tpose.y,cfg->roi_x-tpose.x);
    if (r < cfg->roi_radius) {
        RCLCPP_INFO(node->get_logger(),"Detected ROI at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForROI)
