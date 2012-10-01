
#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/DynamicTask.h"
#include "task_manager_test/TaskIdle.h"

using namespace task_manager_test;

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


void testTSb()
{
    ros::NodeHandle nh("~");
    boost::shared_ptr<TaskEnvironment> env(new TaskEnvironment());
	printf("\n*******************\n\nTesting basic task scheduler functions (B)\n");
	printf("Creating tasks\n");
    boost::shared_ptr<TaskDefinition> idle(new TaskIdle(env));
	printf("Creating task scheduler\n");
	TaskScheduler ts(nh,idle, 1.0);
	ts.printTaskDirectory();
	printf("Loading tasks\n");
	ts.loadTask("./lib/libTaskTest.so",env);
	printf("Configuring tasks\n");
	ts.configureTasks();
	// don't delete tasks, because the ts took responsibility for them
	ts.printTaskDirectory();
	printf("Launching idle task\n");
	ts.startScheduler();
	wait5sec();
	printf("Destroying task scheduler\n");
}


int main(int argc, char * argv[])
{
    ros::init(argc,argv,"client");

	testTSb();

	return 0;
}
