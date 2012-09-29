
#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/DynamicTask.h"
#include "task_manager_test/TaskIdle.h"

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


void testTSc()
{
	TaskEnvironment env;
    ros::NodeHandle nh("~");
	TaskParameters tp1,tp2;
	printf("\n*******************\n\nTesting task scheduler functions (multitask)\n");
	printf("Loading tasks parameters\n");
    tp1.setParameter("task_timeout",10.);
    tp1.setParameter("main_task",false);
    tp2 = tp1;

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	TaskDefinition *dtask1 = new DynamicTask("./lib/libTaskTest.so",&env);
	dtask1->setName("Task1");
	TaskDefinition *dtask2 = new DynamicTask("./lib/libTaskTest.so",&env);
	dtask2->setName("Task2");

	printf("Creating task scheduler\n");
	TaskScheduler ts(nh,idle, 0.5);
	ts.printTaskDirectory();
	printf("Adding tasks\n");
	ts.addTask(dtask1);
	ts.addTask(dtask2);
	ts.printTaskDirectory();
	printf("Configuring tasks\n");
	ts.configureTasks();
	printf("Launching idle task\n");
	ts.startScheduler();
	waitabit(2);
	printf("Launching test tasks in foreground\n");
    tp1.setParameter("task_period",0.5);
	ts.launchTask(dtask1->getName(),tp1);
    tp2.setParameter("task_period",0.7);
	ts.launchTask(dtask2->getName(),tp2);
	waitabit(3);
	// don't delete tasks, because the ts took responsibility for them
	printf("Destroying task scheduler\n");
}

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"client");
	testTSc();
	return 0;
}
