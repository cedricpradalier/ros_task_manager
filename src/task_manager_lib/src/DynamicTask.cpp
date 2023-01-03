#include <dlfcn.h>

#include "task_manager_lib/DynamicTask.h"
using namespace task_manager_lib;


DynamicTask::DynamicTask(const std::string & fname, TaskEnvironmentPtr env) :
	TaskDefinitionBase("DynamicTask","undefined",true,env,TaskConfigPtr()),filename(fname) 
{
	// Hack to garantee alignment on 32 bits, for valgrind
	char fbuffer[1024];
	strncpy(fbuffer,filename.c_str(),1023);
	fbuffer[1023] = 0;

	handle = dlopen(fbuffer, RTLD_NOW);
	if (!handle) {
        RCLCPP_ERROR(node->get_logger(),"dlopen: %s",dlerror());
		std::string error("dlopen returned NULL handle for file '");
		error += filename + "'";
		throw DLLoadError(error);
	}
	TaskFactory tf = (TaskFactory)dlsym(handle, "TaskFactoryObject");
    if (!tf) {
		throw DLLoadError("The TaskFactoryObject is NULL. The task probably missed the DYNAMIC_TASK macro");
    }
	task = tf(env);
	if (!task) {
		dlclose(handle);
		handle = NULL;
		throw DLLoadError("taskfactory returned NULL task");
	}
    // Do not declare parameters. It has been done in the loaded task. The
    // dynamic task has no parameter of its own.
    // cfg_gen = task->cfg_gen;
}

DynamicTask::~DynamicTask()
{
	// printf("Deleting dynamic task: ");
	task.reset();
	dlclose(handle);
	handle = NULL;
}





