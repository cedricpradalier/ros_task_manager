#include "task_manager_lib/TaskSystem.h"
#include <sys/types.h>
#include <sys/wait.h>
using namespace task_manager_msgs::msg;
using namespace task_manager_lib;

TaskIndicator TaskSystem::initialise() 
{
    pid = fork();
    if (pid == 0) {
        // child process
        signal(SIGKILL,SIG_DFL);
        signal(SIGINT,SIG_DFL);
        if (cfg->command_array.empty()) {
            int res = execl("/bin/sh", "sh", "-c", cfg->command.c_str(), (char*)NULL);
            if (res == -1) {
                perror("execl failed: ");
                return TaskStatus::TASK_INITIALISATION_FAILED;
            }
        } else {
            using charP = char *;
            charP * argv = new charP[cfg->command_array.size()+1];
            for (size_t i=0;i<cfg->command_array.size();i++) {
                argv[i] = strdup(cfg->command_array[i].c_str());
            }
            argv[cfg->command_array.size()] = (char*)NULL;
            int res = execv(argv[0],argv);
            if (res == -1) {
                perror("execv failed: ");
                return TaskStatus::TASK_INITIALISATION_FAILED;
            }
        }
    }
    RCLCPP_INFO(getNode()->get_logger(),"Started child process with PID %d",pid);
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
        rclcpp::Time t0 = getNode()->get_clock()->now();
        RCLCPP_INFO(getNode()->get_logger(),"TaskSystem: sending SIGINT to %d",pid);
        kill(pid,SIGINT);
        while (iterate() == TaskStatus::TASK_RUNNING) {
            usleep(100000);
            rclcpp::Time t1 = getNode()->get_clock()->now();
            if ((t1-t0).seconds() > cfg->terminate_time) {
                RCLCPP_WARN(getNode()->get_logger(),"TaskSystem: excalating to SIGKILL");
                kill(pid,SIGKILL);
            } else if ((t1-t0).seconds() > cfg->terminate_time+1) {
                RCLCPP_ERROR(getNode()->get_logger(),"TaskSystem: subprocess %d refuses to die. Good luck...",pid);
                return TaskStatus::TASK_TERMINATED;
            }
        }
        RCLCPP_INFO(getNode()->get_logger(),"TaskSystem: subprocess terminated");
    }
    return TaskStatus::TASK_TERMINATED;
}


// DYNAMIC_TASK(DefinitionWaitDefault);
