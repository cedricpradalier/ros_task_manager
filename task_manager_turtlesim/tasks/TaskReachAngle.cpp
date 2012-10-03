#include <math.h>
#include "TaskReachAngle.h"
#include "task_manager_turtlesim/TaskReachAngleConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;

TaskReachAngle::TaskReachAngle(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskReachAngleConfig>("ReachAngle","Reach a desired destination",true,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskReachAngle::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskReachAngle::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    printf("Task ReachAngle initialisation\n");
    cfg = parameters.toConfig<TaskReachAngleConfig>();
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskReachAngle::iterate()
{
    const turtlesim::Pose & tpose = env->getPose();
    double alpha = remainder(cfg.target-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg.threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_alpha*alpha;
    if (rot > cfg.max_vrot) rot = cfg.max_vrot;
    if (rot < -cfg.max_vrot) rot = -cfg.max_vrot;
    // printf("Cmd v %.2f r %.2f\n",vel,rot);
    env->publishVelocity(0.0 , rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskReachAngle::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskReachAngle);
