#include <math.h>
#include "TaskGoTo.h"
#include "task_manager_turtlesim/TaskGoToConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;


TaskIndicator TaskGoTo::initialise()  {
    initial_pose = env->getPose();
    if (cfg.relative) {
        ROS_INFO("TaskGoTo: Going to (%.2f,%.2f)",
                initial_pose.x
                    + cfg.goal_x*cos(initial_pose.theta)
                    - cfg.goal_y*sin(initial_pose.theta),
                initial_pose.y
                    + cfg.goal_x*sin(initial_pose.theta)
                    + cfg.goal_y*cos(initial_pose.theta));
    } else {
        ROS_INFO("TaskGoTo: Going to (%.2f,%.2f)",cfg.goal_x,cfg.goal_y);
    }
    return TaskStatus::TASK_INITIALISED;
}
            

TaskIndicator TaskGoTo::iterate()
{
    const turtlesim::Pose & tpose = env->getPose();
    double goal_x = cfg.goal_x, goal_y = cfg.goal_y;
    if (cfg.relative) {
        goal_x = initial_pose.x
            + cfg.goal_x*cos(initial_pose.theta)
            - cfg.goal_y*sin(initial_pose.theta);
        goal_y = initial_pose.y
            + cfg.goal_x*sin(initial_pose.theta)
            + cfg.goal_y*cos(initial_pose.theta);
    }
    double r = hypot(goal_y-tpose.y,goal_x-tpose.x);
    if ((tpose.x < 0.1) && (goal_x < tpose.x)) {
		return TaskStatus::TASK_COMPLETED;
    }
    if ((tpose.y < 0.1) && (goal_y < tpose.y)) {
		return TaskStatus::TASK_COMPLETED;
    }
    if ((tpose.x > 10.9) && (goal_x > tpose.x)) {
		return TaskStatus::TASK_COMPLETED;
    }
    if ((tpose.y > 10.9) && (goal_y > tpose.y)) {
		return TaskStatus::TASK_COMPLETED;
    }
    if (r < cfg.dist_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((goal_y-tpose.y),goal_x-tpose.x)-tpose.theta,2*M_PI);
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
