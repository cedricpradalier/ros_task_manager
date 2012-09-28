
#include "TaskIdle.h"
#include "DynamicTask.h"

int main()
{
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

	TaskDefinition *dtask = new DynamicTask("./libTaskTest.so",&env);
	dtask->doConfigure(tp);
	dtask->doInitialise(tp);
	while (dtask->getStatus() != TASK_COMPLETED) {
		dtask->doIterate();
	}
	dtask->doTerminate();
	delete dtask;

	return 0;
}
	



