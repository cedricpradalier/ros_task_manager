
#include "TaskScheduler.h"
#include "DynamicTask.h"
#include "TaskIdle.h"

void waitabit(unsigned int bit)
{
	printf("Waiting %d bits:",bit); fflush(stdout);
	for (unsigned int i=0;i<bit;i++) {
		sleep(1);
		//printf(".");
		//fflush(stdout);
	}
	printf("\n");
}


void testTSa()
{
	TaskEnvironment env;
	TaskParameters tp;
	printf("\n*******************\n\nTesting task scheduler functions (foreground)\n");
	printf("Loading tasks parameters\n");
	// tp.loadFromString(config);
	tp.setDoubleParam("task_timeout",10);
	tp.setDoubleParam("task_period",0.5);
	tp.setLongParam("main_task",1);

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	TaskDefinition *dtask = new DynamicTask("./libTaskTest.so",&env);
	printf("Creating task scheduler\n");
	TaskScheduler ts(idle, 0.5);
	ts.printTaskDirectory();
	printf("Adding tasks\n");
	ts.addTask(dtask);
	ts.printTaskDirectory();
	printf("Configuring tasks\n");
	ts.configureTasks("../../tests","cfg");
	// don't delete tasks, because the ts took responsibility for them
	printf("Launching idle task\n");
	ts.startScheduler();
	waitabit(3);
	printf("Launching test task in foreground\n");
	ts.launchTask(dtask->getName(),tp);
	waitabit(3);
	printf("Destroying task scheduler\n");
}



int main()
{

	testTSa();
	return 0;
}
