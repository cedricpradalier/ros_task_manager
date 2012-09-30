

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskClient.h"

#define DEBUG(c) printf("Executing "#c":");res=c;printf("%d\n",res);
#define DEBUGSTATUS(c) c

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

	TaskClient client("/tasks",nh);

	printf("Task list on the server:\n");
	client.printTaskList();

	printf("Task status on the server:\n");
	client.printStatusMap();

    // sleep(1);

	printf("Running task Test\n");
    tp.setParameter("task_duration",5.0);
	DEBUG(client.startTask("Test",true,0.5,tp));
	printf("Task status on the server:\n");
	client.printStatusMap();

	printf("Sleeping 3 sec\n");
	for (i=0;i<3;i++) {
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}


	printf("Back to idle\n");
	DEBUG(client.idle());
	for (i=0;i<10;i++) {
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}
	printf("Terminating\n");
}

