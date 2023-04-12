#include <math.h>
#include "TaskReachAngle.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskReachAngle::initialise()
{
    const turtlesim::msg::Pose & tpose = env->getPose();
    initial_theta = tpose.theta;
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskReachAngle::iterate()
{
    const turtlesim::msg::Pose & tpose = env->getPose();
    double target = cfg->relative?(initial_theta+cfg->target):cfg->target;
    double alpha = remainder(target-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg->threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg->k_alpha*alpha;
    if (rot > cfg->max_vrot) rot = cfg->max_vrot;
    if (rot < -cfg->max_vrot) rot = -cfg->max_vrot;
    // printf("Cmd v %.2f r %.2f\n",vel,rot);
    env->publishVelocity(0.0 , rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskReachAngle::terminate()
{
    env->publishVelocity(0,0);
	return Parent::terminate();
}

DYNAMIC_TASK(TaskFactoryReachAngle)
