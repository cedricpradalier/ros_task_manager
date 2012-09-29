

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskClient.h"

#define DEBUG(c) printf("Executing "#c":");res=c;printf("%d\n",res);
#define DEBUGSTATUS(c) printf("Executing "#c);

int end = 0;
void sighdl(int n) {
	end ++;
}

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"client");
    ros::NodeHandle nh("~");
	int i,res = -1;
	TaskParameters tp;
	TaskClient::StatusMap sm;
	signal(SIGINT,sighdl);

	TaskClient client("/tasks",nh);

	DEBUG(client.updateTaskList());

	printf("Task list on the server:\n");
	client.printTaskList();

	DEBUGSTATUS(client.updateAllStatus());
	printf("Task status on the server:\n");
	client.printStatusMap();

	printf("Running task Test\n");
	DEBUG(client.startTask("Test",true,1.0,tp));
	DEBUGSTATUS(client.updateAllStatus());
	printf("Task status on the server:\n");
	client.printStatusMap();

	printf("Sleeping 5 sec\n");
	for (i=0;i<5;i++) {
		DEBUGSTATUS(client.updateAllStatus());
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}


	printf("Back to idle\n");
	DEBUG(client.idle());
	for (i=0;i<3;i++) {
		DEBUGSTATUS(client.updateAllStatus());
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}
	printf("Terminating\n");
}

