#include <math.h>
#include "TaskWaitForButton.h"
#include "boost/algorithm/string.hpp"
#include "task_manager_turtlesim/TaskWaitForButtonConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;
using namespace boost::algorithm;

TaskIndicator TaskWaitForButton::initialise()
{
    button_sub = env->getNodeHandle().subscribe("/buttons",10,&TaskWaitForButton::buttonCallback,this);
    triggered = false;
    expected_string.clear();
    std::vector<std::string> splitted;
    split( splitted, cfg.text, is_any_of("|"), token_compress_on );
    for (unsigned int i=0;i<splitted.size();i++) {
        expected_string.insert(to_lower_copy(splitted[i]));
    }
    ROS_INFO("%p: Waiting for button: '%s'",this,cfg.text.c_str());
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForButton::iterate()
{
    if (triggered) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryWaitForButton);
