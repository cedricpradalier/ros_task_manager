
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


void testTSd()
{
	TaskEnvironment env;
    ros::NodeHandle nh("~");
	TaskParameters tp;
	printf("\n*******************\n\nTesting task scheduler functions (timeout)\n");
	printf("Loading tasks parameters\n");
	// tp.loadFromString(config);
    tp.setParameter("task_timeout",5.);
    tp.setParameter("task_period",0.5);
    tp.setParameter("task_duration",50.);
    tp.setParameter("main_task",true);

	printf("Creating tasks\n");
	TaskDefinition *idle = new TaskIdle(&env);
	TaskDefinition *dtask = new DynamicTask("./lib/libTaskTest.so",&env);
	printf("Creating task scheduler\n");
	TaskScheduler ts(nh,idle, 0.5);
	ts.printTaskDirectory();
	printf("Adding tasks\n");
	ts.addTask(dtask);
	ts.printTaskDirectory();
	printf("Configuring tasks\n");
	ts.configureTasks();
	printf("Launching idle task\n");
	ts.startScheduler();
	waitabit(2);
	printf("Launching test tasks in foreground, with timeout\n");
	ts.launchTask(dtask->getName(),tp);
	waitabit(10);
	// don't delete tasks, because the ts took responsibility for them
	printf("Destroying task scheduler\n");
}

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"client");
	testTSd();
	return 0;
}
