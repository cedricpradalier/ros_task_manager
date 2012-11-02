#include <math.h>
#include "TaskWaitForButton.h"
#include "boost/algorithm/string.hpp"
#include "task_manager_turtlesim/TaskWaitForButtonConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;
using namespace boost::algorithm;

TaskWaitForButton::TaskWaitForButton(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskWaitForButtonConfig>("WaitForButton","Do nothing until we receive a button message",true,-1.)
{
    env = boost::dynamic_pointer_cast<TurtleSimEnv,TaskEnvironment>(tenv);
    button_sub = env->getNodeHandle().subscribe("/buttons",10,&TaskWaitForButton::buttonCallback,this);
    triggered = false;
}

TaskIndicator TaskWaitForButton::configure(const TaskParameters & parameters) throw (InvalidParameter)
{
	return TaskStatus::TASK_CONFIGURED;
}

TaskIndicator TaskWaitForButton::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    cfg = parameters.toConfig<TaskWaitForButtonConfig>();
    triggered = false;
    expected_string.clear();
    std::vector<std::string> splitted;
    split( splitted, cfg.text, is_any_of("|"), token_compress_on );
    for (unsigned int i=0;i<splitted.size();i++) {
        expected_string.insert(to_lower_copy(splitted[i]));
    }
    ROS_INFO("Waiting for button: '%s'",cfg.text.c_str());
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForButton::iterate()
{
    if (triggered) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForButton::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskWaitForButton);
