#include "task_manager_lib/TaskSystem.h"
#include <sys/types.h>
#include <sys/wait.h>
using namespace task_manager_msgs;
using namespace task_manager_lib;

TaskIndicator TaskSystem::initialise() 
{
    pid = fork();
    if (pid == 0) {
        // child process
        int res = execl("/bin/sh", "sh", "-c", cfg.command.c_str(), (char*)NULL);
        if (res == -1) {
            perror("execl failed: ");
            return TaskStatus::TASK_INITIALISATION_FAILED;
        }
    }
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskSystem::iterate()
{
    int status=0;
    pid_t rval = waitpid(pid, &status, WNOHANG);
    
    if (rval == -1) {
        pid = 0;
        perror("waitpid failed: ");
        return TaskStatus::TASK_FAILED;
    } else if (rval == 0) {
        return TaskStatus::TASK_RUNNING;
    } else {
        pid = 0;
        if (WIFEXITED(status) && (WEXITSTATUS(status)==0)) {
            return TaskStatus::TASK_COMPLETED;
        }
        return TaskStatus::TASK_FAILED;
    }

}


TaskIndicator TaskSystem::terminate()
{
    if (pid != 0) {
        ros::Time t0 = ros::Time::now();
        ROS_INFO("TaskSystem: sending SIGINT");
        kill(pid,SIGINT);
        while (iterate() == TaskStatus::TASK_RUNNING) {
            usleep(100000);
            ros::Time t1 = ros::Time::now();
            if ((t1-t0).toSec() > cfg.terminate_time) {
                ROS_WARN("TaskSystem: excalating to SIGKILL");
                kill(pid,SIGKILL);
                return TaskStatus::TASK_TERMINATED;
            }
        }
        ROS_INFO("TaskSystem: subprocess terminated");
    }
    return TaskStatus::TASK_TERMINATED;
}


// DYNAMIC_TASK(DefinitionWaitDefault);
