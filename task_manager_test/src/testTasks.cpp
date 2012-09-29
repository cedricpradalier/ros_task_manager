
#include "task_manager_test/TaskIdle.h"
#include "task_manager_lib/DynamicTask.h"

using namespace task_manager_msgs;
using namespace task_manager_test;


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tasks");
    ros::NodeHandle nh("~");
	unsigned int i;
	TaskEnvironment env;
	TaskParameters tp;
	TaskDefinition *idle = new TaskIdle(&env);
	idle->doConfigure(tp);
	idle->doInitialise(tp);
	for (i=0;i<5;i++) {
		idle->doIterate();
	}
	idle->doTerminate();
	delete idle;

	TaskDefinition *dtask = new DynamicTask("./lib/libTaskTest.so",&env);
	dtask->doConfigure(tp);
	dtask->doInitialise(tp);
	while (dtask->getStatus() != TaskStatus::TASK_COMPLETED) {
		dtask->doIterate();
	}
	dtask->doTerminate();
	delete dtask;

	return 0;
}
	



