
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


void testTSd()
{
	TaskEnvironment env;
	TaskParameters tp;
	printf("\n*******************\n\nTesting task scheduler functions (timeout)\n");
	printf("Loading tasks parameters\n");
	// tp.loadFromString(config);
	tp.setDoubleParam("task_timeout",5);
	tp.setDoubleParam("task_period",0.5);
	tp.setLongParam("task_duration",50);
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
	printf("Launching idle task\n");
	ts.startScheduler();
	waitabit(2);
	printf("Launching test tasks in foreground, with timeout\n");
	ts.launchTask(dtask->getName(),tp);
	waitabit(10);
	// don't delete tasks, because the ts took responsibility for them
	printf("Destroying task scheduler\n");
}

int main()
{

	testTSd();
	return 0;
}
