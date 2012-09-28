#include <sys/time.h>
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
#include "TaskDefinition.h"
#include "TaskServer.h"
#include "TaskScheduler.h"
#include "TaskIdle.h"

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)



int main(int argc, const char *argv[])
{
	int res;
	SBApiSimpleContext simple;
	TaskCoaxEnvironment env(&simple);
	TaskDefinition *idle = new TaskIdle(&env);

	TaskScheduler ts(idle, 0.1);
	ts.loadAllTasks(".",&env);

	res = 0;
	sbSimpleDefaultContext(&simple);
	std::string devname = (argc<2)?("localhost"):(argv[1]);
	simple.device[0] = 0;
	strncpy(simple.device,devname.c_str(),255);
	simple.device[255] = 0;
	simple.commPort = 5123; // 8123 to test the repeater
	if (simple.device[0] == '/') {
		simple.commType = SB_COMMTYPE_SERIAL;
	} else {
		simple.commType = SB_COMMTYPE_UDP;
		sscanf(devname.c_str(),"%[^:]:%d",simple.device,&simple.commPort);
	}
	simple.initNavState = SB_NAV_STOP;
	simple.cmdTimeout = 0xffff; // no timeout... dangerous
	simple.ctrlTimeout = 10000;
	simple.sensorList = &env.sensorList;
	simple.rollCtrlMode = SB_CTRL_POS;
	simple.pitchCtrlMode = SB_CTRL_POS;

	DEBUG(res = sbSimpleInitialise(&simple));
	printf("Channel connected, continuing\n");

	printf("Configuring tasks\n");
	ts.configureTasks(".","cfg");
	printf("Launching idle task\n");
	ts.startScheduler();

	TaskServer server(ts);
	server.initialiseServer(TaskServer::TS_PORT);

	while (!*(simple.endP)) {
		sleep(1);
	}
	server.terminateServer();

	DEBUG(res = sbSimpleReachNavState(&simple,SB_NAV_STOP,20.0));
	DEBUG(res = sbSimpleTerminate(&simple));

	return 0;
}

		
