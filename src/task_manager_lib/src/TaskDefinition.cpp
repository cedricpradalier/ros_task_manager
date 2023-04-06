
// #include <dynamic_reconfigure/config_tools.h>
#include "task_manager_lib/TaskDefinition.h"
using namespace task_manager_lib;
using std::placeholders::_1;




void TaskDefinitionBase::setName(const std::string & n) {
	name = n;
}

void TaskDefinitionBase::setTaskId(int id) {
	taskId = id;
}

int TaskDefinitionBase::getTaskId() const {
    return taskId;
}

const std::string & TaskDefinitionBase::getName() const {
	return name;
}

const std::string & TaskDefinitionBase::getHelp() const {
	return help;
}

bool TaskDefinitionBase::isPeriodic() const {
	return periodic;
}

void TaskDefinitionBase::debug(const char *stemplate,...) const {
	va_list args;
    char buffer[1024];
	va_start(args, stemplate);
	vsnprintf(buffer,1023, stemplate,args);
	va_end(args);
    buffer[1023]=0;
    RCLCPP_INFO(node->get_logger(),"%s: %s",this->getName().c_str(),buffer);
}

task_manager_msgs::msg::TaskDescription TaskDefinitionBase::getDescription() const {
    task_manager_msgs::msg::TaskDescription td;
    td.name = this->getName();
    td.description = this->getHelp();
    td.periodic = this->isPeriodic();
    td.config = this->getConfig()->getParameterDescriptions();
    return td;
}




const char * task_manager_lib::taskStatusToString(TaskIndicator ts)
{
    unsigned int te = task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
    if (ts == te) {
		return "TERMINATED";
    } else if (ts & te) {
        // Terminated
        ts = ts & (~te);
        switch (ts) {
            case task_manager_msgs::msg::TaskStatus::TASK_COMPLETED: return "TERMINATED:COMPLETED";
            case task_manager_msgs::msg::TaskStatus::TASK_FAILED: return "TERMINATED:FAILED";
            case task_manager_msgs::msg::TaskStatus::TASK_INTERRUPTED: return "TERMINATED:INTERRUPTED";
            case task_manager_msgs::msg::TaskStatus::TASK_TIMEOUT: return "TERMINATED:TIMEOUT";
            case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURATION_FAILED: return "TERMINATED:CONFIGURATION FAILED";
            case task_manager_msgs::msg::TaskStatus::TASK_INITIALISATION_FAILED: return "TERMINATED:INITIALISATION FAILED";
            default: return "INVALID STATUS";
        }
    } else {
        // Not terminated
        switch (ts) {
            case task_manager_msgs::msg::TaskStatus::TASK_NEWBORN: return "NEWBORN"; 
            case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED: return "CONFIGURED";
            case task_manager_msgs::msg::TaskStatus::TASK_INITIALISED: return "INITIALISED";
            case task_manager_msgs::msg::TaskStatus::TASK_RUNNING: return "RUNNING";
            case task_manager_msgs::msg::TaskStatus::TASK_COMPLETED: return "COMPLETED";
            case task_manager_msgs::msg::TaskStatus::TASK_FAILED: return "FAILED";
            case task_manager_msgs::msg::TaskStatus::TASK_TIMEOUT: return "TIMEOUT";
            case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURATION_FAILED: return "CONFIGURATION FAILED";
            case task_manager_msgs::msg::TaskStatus::TASK_INITIALISATION_FAILED: return "INITIALISATION FAILED";
            default: return "INVALID STATUS";
        }
    }
	return NULL;
}

