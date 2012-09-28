

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "TaskClient.h"

#define DEBUG(c) printf("Executing "#c":");res=c;printf("%d\n",res);
#define DEBUGSTATUS(c) printf("Executing "#c":");sm=c;printf("%d\n",sm.size());

int end = 0;
void sighdl(int n) {
	end ++;
}

int main()
{
	int i,res = -1;
	TaskParameters tp;
	TaskClient::StatusMap sm;
	signal(SIGINT,sighdl);

	TaskClient client("localhost");
	DEBUG(client.open());

	DEBUG(client.updateTaskList());

	printf("Task list on the server:\n");
	client.printTaskList();

	DEBUGSTATUS(client.getStatus());
	printf("Task status on the server:\n");
	client.printStatusMap();

	printf("Running task Test\n");
	DEBUG(client.startTask("Test",false,0.5,tp));
	DEBUGSTATUS(client.getStatus());
	printf("Task status on the server:\n");
	client.printStatusMap();

	printf("Sleeping 3 sec\n");
	for (i=0;i<3;i++) {
		DEBUGSTATUS(client.getStatus());
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}


	printf("Back to idle\n");
	DEBUG(client.idle());
	for (i=0;i<3;i++) {
		DEBUGSTATUS(client.getStatus());
		printf("Task status on the server:\n");
		client.printStatusMap();
		sleep(1);
	}
	printf("Terminating\n");
}

