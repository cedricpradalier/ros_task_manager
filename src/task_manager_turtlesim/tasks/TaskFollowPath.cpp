#include <math.h>
#include "TaskFollowPath.h"
#include "task_manager_turtlesim/TaskFollowPathConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskFollowPath::initialise()  {
    switch (cfg.param_list_action) {
        case FollowPathPL::Clear:
            env->getWP().clear();
            ROS_INFO("TaskFollowPath: Clear");
            break;
        case FollowPathPL::Push:
            env->getWP().push_config(cfg);
            ROS_INFO("TaskFollowPath: Push");
            break;
        case FollowPathPL::Execute:
            ROS_INFO("TaskFollowPath: Start");
            break;
    }
    return TaskStatus::TASK_INITIALISED;
}
            

TaskIndicator TaskFollowPath::iterate()
{
    if (cfg.param_list_action != FollowPathPL::Execute) {
        return TaskStatus::TASK_COMPLETED;
    }
    const turtlesim::Pose & tpose = env->getPose();
    float goal_x = 0, goal_y = 0, r = 0;
    while (1) {
        if (env->getWP().empty()) {
            return TaskStatus::TASK_COMPLETED;
        }
        env->getWP().collect_front(goal_x, goal_y);
        if ((tpose.x < 0.1) && (goal_x < tpose.x)) {
            env->popWP();
            continue;
        }
        if ((tpose.y < 0.1) && (goal_y < tpose.y)) {
            env->popWP();
            continue;
        }
        if ((tpose.x > 10.9) && (goal_x > tpose.x)) {
            env->popWP();
            continue;
        }
        if ((tpose.y > 10.9) && (goal_y > tpose.y)) {
            env->popWP();
            continue;
        }
        r = hypot(goal_y-tpose.y,goal_x-tpose.x);
        if (r < cfg.dist_threshold) {
            env->popWP();
            continue;
        }
        break;
    }
    double alpha = remainder(atan2((goal_y-tpose.y),goal_x-tpose.x)-tpose.theta,2*M_PI);
    if (fabs(alpha) > M_PI/6) {
        double rot = ((alpha>0)?+1:-1)*M_PI/6;
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        double rot = cfg.k_alpha*alpha;
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
        env->publishVelocity(vel, rot);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFollowPath::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryFollowPath);
