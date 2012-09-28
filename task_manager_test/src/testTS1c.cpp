
#include "TaskScheduler.h"
#include "DynamicTask.h"
#include "TaskIdle.h"

void wait5sec()
{
	printf("Waiting:"); fflush(stdout);
	for (unsigned int i=0;i<5;i++) {
		sleep(1);
		printf(".");
		fflush(stdout);
	}
	printf("\n");
}



void testTSc()
{
	TaskEnvironment env;
	printf("\n*******************\n\nTesting basic task scheduler functions (C)\n");
	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	printf("Creating task scheduler\n");
	TaskScheduler ts(idle, 1.0);
	ts.printTaskDirectory();
	printf("Scanning tasks directory\n");
	ts.loadAllTasks(".",&env);
	printf("Configuring tasks\n");
	ts.configureTasks("../../tests","cfg");
	// don't delete tasks, because the ts took responsibility for them
	ts.printTaskDirectory();
	printf("Launching idle task\n");
	ts.startScheduler();
	wait5sec();
	printf("Destroying task scheduler\n");
}



int main()
{
	testTSc();
	return 0;
}
