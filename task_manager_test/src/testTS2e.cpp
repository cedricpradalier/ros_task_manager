
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


void testTSe()
{
	TaskEnvironment env;
	TaskParameters tp;
	printf("\n*******************\n\nTesting task scheduler functions (sequence)\n");
	printf("Loading tasks parameters\n");
	// tp.loadFromString(config);
	tp.setDoubleParam("task_timeout",10);
	tp.setLongParam("main_task",1);

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	TaskDefinition *dtask1 = new DynamicTask("./libTaskTest.so",&env);
	dtask1->setName("Task1");
	TaskDefinition *dtask2 = new DynamicTask("./libTaskTest.so",&env);
	dtask2->setName("Task2");

	printf("Creating task scheduler\n");
	TaskScheduler ts(idle, 0.5);
	ts.printTaskDirectory();
	printf("Adding tasks\n");
	ts.addTask(dtask1);
	ts.addTask(dtask2);
	ts.printTaskDirectory();
	printf("Configuring tasks\n");
	ts.configureTasks("../../tests","cfg");
	printf("Launching idle task\n");
	ts.startScheduler();
	waitabit(2);
	printf("Launching test tasks in foreground\n");
	tp.setDoubleParam("task_period",0.5);
	TaskScheduler::TaskId id1 = ts.launchTask(dtask1->getName(),tp);
	printf("Task id: %d\n",(int)id1);
	ts.waitTaskCompletion(id1,50.0);
	printf("Task completed\n");


	tp.setDoubleParam("task_period",0.7);
	TaskScheduler::TaskId id2 = ts.launchTask(dtask2->getName(),tp);
	printf("Task id: %d\n",(int)id2);
	ts.waitTaskCompletion(id2,50.0);
	printf("Task completed\n");
	waitabit(1);
	// don't delete tasks, because the ts took responsibility for them
	printf("Destroying task scheduler\n");
}


int main()
{
	testTSe();

	return 0;
}
