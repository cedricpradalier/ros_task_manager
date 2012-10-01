#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/DynamicTask.h"
#include "task_manager_lib/TaskIdleDefault.h"
#include "task_manager_lib/TaskWaitDefault.h"

#include "task_manager_turtlesim/TurtleSimEnv.h"

int end = 0;

void sighdl(int n) {
	end ++;
}


using namespace task_manager_turtlesim;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"turtlesim_tasks");
    ros::NodeHandle nh("~");
    std::string lib_path = "./lib";
    nh.getParam("lib_path",lib_path);

    boost::shared_ptr<TaskEnvironment> env(new TurtleSimEnv(nh,1));
    boost::shared_ptr<TaskDefinition> idle(new task_manager_lib::TaskIdleDefault(env));
    boost::shared_ptr<TaskDefinition> wait(new task_manager_lib::TaskWaitDefault(env));
	TaskScheduler ts(nh, idle, 0.5);
    ts.addTask(wait);
	ts.loadAllTasks(lib_path,env);
	ts.configureTasks();
	ts.printTaskDirectory(true);
	ts.startScheduler();

    ros::spin();

	return 0;
}
