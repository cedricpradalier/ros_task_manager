#include <dlfcn.h>

#include "task_manager_lib/DynamicTask.h"
using namespace task_manager_lib;


DynamicTask::DynamicTask(const std::string & fname, TaskEnvironmentPtr env) :
	TaskDefinitionBase("DynamicTask","undefined",true,env,TaskConfigPtr()),filename(fname) {
        task.reset();
        handle = NULL;
    }

bool DynamicTask::loadTask(bool exceptions) {
	// Hack to garantee alignment on 32 bits, for valgrind
	char fbuffer[1024];
	strncpy(fbuffer,filename.c_str(),1023);
	fbuffer[1023] = 0;

	handle = dlopen(fbuffer, RTLD_NOW);
	if (!handle) {
        RCLCPP_ERROR(node->get_logger(),"dlopen: %s",dlerror());
		std::string error("dlopen returned NULL handle for file '");
		error += filename + "'";
        if (exceptions) {
            throw DLLoadError(error);
        } else {
            RCLCPP_ERROR(node->get_logger(),"%s",error.c_str());
            return false;
        }
	}
	EnvCheckSum ef = (EnvCheckSum)dlsym(handle, "getEnvironmentCheckSum");
    if (!ef) {
		dlclose(handle);
        handle = NULL;
        if (exceptions) {
            throw DLLoadError("The getEnvironmentCheckSum is NULL. The task probably missed the DYNAMIC_TASK macro");
        } else {
            RCLCPP_ERROR(node->get_logger(),"The getEnvironmentCheckSum is NULL. The task probably missed the DYNAMIC_TASK macro");
            return false;
        }
    }
    RCLCPP_DEBUG(node->get_logger(),"Task %s Env check sum: %s",filename.c_str(),ef()); 
    RCLCPP_DEBUG(node->get_logger(),"Env check sum: %s",env_gen->getEnvironmentCheckSum()); 
    if (std::string(ef()) != std::string(env_gen->getEnvironmentCheckSum())) {
        RCLCPP_INFO(node->get_logger(),"Task %s Env check sum: %s",filename.c_str(),ef()); 
        RCLCPP_INFO(node->get_logger(),"Env check sum: %s",env_gen->getEnvironmentCheckSum()); 
		dlclose(handle);
        handle = NULL;
        if (exceptions) {
            throw DLLoadError("The environment checksum is inconsistent. The task has been compiled with a different environment.");
        } else {
            RCLCPP_ERROR(node->get_logger(),"The environment checksum is inconsistent. The task has been compiled with a different environment.");
            return false;
        }
    }
	TaskFactory tf = (TaskFactory)dlsym(handle, "TaskFactoryObject");
    if (!tf) {
		dlclose(handle);
        handle = NULL;
        if (exceptions) {
            throw DLLoadError("The TaskFactoryObject is NULL. The task probably missed the DYNAMIC_TASK macro");
        } else {
            RCLCPP_ERROR(node->get_logger(),"The TaskFactoryObject is NULL. The task probably missed the DYNAMIC_TASK macro");
            return false;
        }
    }
	task = tf(env_gen);
	if (!task) {
		dlclose(handle);
		handle = NULL;
        if (exceptions) {
            throw DLLoadError("taskfactory returned NULL task");
        } else {
            RCLCPP_ERROR(node->get_logger(),"taskfactory returned NULL task");
            return false;
        }
	}
    // Do not declare parameters. It has been done in the loaded task. The
    // dynamic task has no parameter of its own.
    // cfg_gen = task->cfg_gen;
    return true;
}

DynamicTask::~DynamicTask()
{
	// printf("Deleting dynamic task: ");
	task.reset();
    if (handle) {
        dlclose(handle);
        handle = NULL;
    }
}





