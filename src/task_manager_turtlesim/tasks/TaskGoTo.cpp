#include <math.h>
#include "TaskGoTo.h"
#include "task_manager_turtlesim/TaskGoToConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskGoTo::initialise()  {
    ROS_INFO("TaskGoTo: Going to (%.2f,%.2f)",cfg.goal_x,cfg.goal_y);
    return TaskStatus::TASK_INITIALISED;
}
            

TaskIndicator TaskGoTo::iterate()
{
    const turtlesim::Pose & tpose = env->getPose();
    double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);
    if (r < cfg.dist_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((cfg.goal_y-tpose.y),cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
    // printf("g %.1f %.1f r %.3f alpha %.1f\n",cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
    if (fabs(alpha) > M_PI/6) {
        double rot = ((alpha>0)?+1:-1)*M_PI/6;
        // printf("Cmd v %.2f r %.2f\n",0.,rot);
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        double rot = cfg.k_alpha*alpha;
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
        // printf("Cmd v %.2f r %.2f\n",vel,rot);
        env->publishVelocity(vel, rot);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoTo::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoTo);
