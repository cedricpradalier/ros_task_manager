#include <math.h>
#include "TaskWaitForButton.h"
#include "boost/algorithm/string.hpp"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace task_manager_turtlesim;
using namespace boost::algorithm;

TaskIndicator TaskWaitForButton::initialise()
{
    button_sub = node->create_subscription<std_msgs::msg::String>("/buttons",1,
            std::bind(&TaskWaitForButton::buttonCallback,this,std::placeholders::_1));
    triggered = false;
    expected_string.clear();
    std::vector<std::string> splitted;
    split( splitted, cfg->get<std::string>("text"), is_any_of("|"), token_compress_on );
    for (unsigned int i=0;i<splitted.size();i++) {
        expected_string.insert(to_lower_copy(splitted[i]));
    }
    RCLCPP_INFO(node->get_logger(),"%p: Waiting for button: '%s'",(void*)this,cfg->get<std::string>("text").c_str());
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForButton::iterate()
{
    if (triggered) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryWaitForButton)
