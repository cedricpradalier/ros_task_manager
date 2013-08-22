
#include "task_manager_test/TaskIdle.h"
#include "task_manager_lib/DynamicTask.h"

using namespace task_manager_msgs;
using namespace task_manager_test;


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tasks");
    ros::NodeHandle nh("~");
	unsigned int i;
	TaskParameters tp;
    boost::shared_ptr<TaskEnvironment> env(new TaskEnvironment());
    boost::shared_ptr<TaskDefinition> idle(new TaskIdle(env));
	idle->doConfigure(0,tp);
	idle->doInitialise(0,tp);
	for (i=0;i<5;i++) {
		idle->doIterate();
	}
	idle->doTerminate();
    idle.reset();

    boost::shared_ptr<TaskDefinition> dtask(new DynamicTask("./lib/libTaskTest.so",env));
	dtask->doConfigure(1,tp);
	dtask->doInitialise(1,tp);
	while (dtask->getStatus() != TaskStatus::TASK_COMPLETED) {
		dtask->doIterate();
	}
	dtask->doTerminate();
	dtask.reset();

	return 0;
}
	



