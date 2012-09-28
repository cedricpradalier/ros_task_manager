#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "TaskServer.h"
#include "TaskScheduler.h"
#include "DynamicTask.h"
#include "TaskIdle.h"

int end = 0;

void sighdl(int n) {
	end ++;
}


int main()
{
	TaskEnvironment env;
	TaskParameters tp;
	signal(SIGINT,sighdl);

	printf("\n*******************\n\nTesting task server functions\n");
	printf("Loading tasks parameters\n");
	// tp.loadFromString(config);
	tp.setDoubleParam("task_timeout",10);
	tp.setDoubleParam("task_period",0.5);
	tp.setLongParam("main_task",1);

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	printf("Creating task scheduler\n");
	TaskScheduler ts(idle, 0.5);
	ts.printTaskDirectory();
	printf("Scanning tasks directory\n");
	ts.loadAllTasks(".",&env);
	printf("Configuring tasks\n");
	ts.configureTasks("../../tests","cfg");
	// don't delete tasks, because the ts took responsibility for them
	printf("Launching idle task\n");
	ts.startScheduler();

	TaskServer server(ts);
	server.initialiseServer(TaskServer::TS_PORT);

	while (!end) {
		sleep(1);
	}
	server.terminateServer();

	return 0;
}
