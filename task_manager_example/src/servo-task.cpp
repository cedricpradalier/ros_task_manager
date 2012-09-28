#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define sscanf sscanf_s
#define strcpy strcpy_s
#define strncpy strncpy_s
#endif

#include <string>
#include <vector>

#include <com/sbapi.h>
#include <com/sbsimple.h>
#include "TaskClient.h"

#define DEBUG(c) printf("Executing "#c":");res=c;printf("%d\n",res);
#define DEBUGSTATUS(c) printf("Executing "#c":");sm=c;printf("%d\n",sm.size());

int end = 0;
void sighdl(int n) {
	end ++;
}



int main(int argc, const char *argv[])
{
	int res = -1;
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
	client.startTaskAndWait("TakeOff",tp);

#if 1
	tp.clear();
	tp.setDoubleParam("altitude",0.4);
	client.startTaskAndWait("Altitude",tp);
#endif

	tp.clear();
	tp.setDoubleParam("distance",0.5);
	tp.setLongParam("sensor",1);
	client.startTaskAndWait("Servo",tp);

#if 1
	tp.clear();
	tp.setDoubleParam("distance",0.9);
	tp.setLongParam("sensor",1);
	client.startTaskAndWait("Servo",tp);

	tp.clear();
	tp.setDoubleParam("yaw",90);
	client.startTaskAndWait("Yaw",tp);
#endif

	tp.clear();
	client.startTaskAndWait("Land",tp);

	printf("Back to idle\n");
	DEBUG(client.idle());
	printf("Terminating\n");
}

		
