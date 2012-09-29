#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/DynamicTask.h"
#include "task_manager_test/TaskIdle.h"

int end = 0;

void sighdl(int n) {
	end ++;
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tasks");
    ros::NodeHandle nh("~");

	TaskEnvironment env;
	printf("\n*******************\n\nTesting task server functions\n");

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	printf("Creating task scheduler\n");
	TaskScheduler ts(nh, idle, 0.5);
	ts.printTaskDirectory();
	printf("Scanning tasks directory\n");
	ts.loadAllTasks("./lib",&env);
	printf("Configuring tasks\n");
	ts.configureTasks();
	// don't delete tasks, because the ts took responsibility for them
	printf("Launching idle task\n");
	ts.startScheduler();

    ros::spin();

	return 0;
}
