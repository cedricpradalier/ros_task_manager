#include <math.h>
#include "TaskWaitForROI.h"
#include "task_manager_turtlesim/TaskWaitForROIConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskWaitForROI::TaskWaitForROI(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskWaitForROIConfig>("WaitForROI","Do nothing until we reach a given destination",true,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskWaitForROI::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskWaitForROI::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    cfg = parameters.toConfig<TaskWaitForROIConfig>();
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForROI::iterate()
{
    const turtlesim::Pose & tpose = env->getPose();
    double r = hypot(cfg.roi_y-tpose.y,cfg.roi_x-tpose.x);
    if (r < cfg.roi_radius) {
        ROS_INFO("Detected ROI at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForROI::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskWaitForROI);
