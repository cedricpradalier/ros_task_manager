
#include <dynamic_reconfigure/config_tools.h>
#include "task_manager_lib/TaskDefinition.h"

void TaskDefinition::setName(const std::string & n) {
	name = n;
}

const std::string & TaskDefinition::getName() const {
	return name;
}

const std::string & TaskDefinition::getHelp() const {
	return help;
}

bool TaskDefinition::isPeriodic() const {
	return periodic;
}

double TaskDefinition::getTimeout() const {
	return timeout;
}

TaskIndicator TaskDefinition::getStatus() const {
	// printf("TaskDefinition: status of %s: %s\n",name.c_str(),taskStatusToString(taskStatus));
	return taskStatus;
}

task_manager_msgs::TaskStatus TaskDefinition::getRosStatus() const {
    task_manager_msgs::TaskStatus st;
    st.name = name;
    st.status = getStatus();
    st.status_string = getStatusString();
    st.plist = (dynamic_reconfigure::Config)config;
    return st;
}

task_manager_msgs::TaskDescription TaskDefinition::getDescription() const {
    task_manager_msgs::TaskDescription td;
    td.name = name;
    td.description = help;
    td.periodic = periodic;
    td.timeout_s = defaultTimeout;
    td.config = parameterDescription;
    return td;
}

void TaskDefinition::resetStatus() {
	// printf("TaskDefinition: status of %s: %s\n",name.c_str(),taskStatusToString(taskStatus));
	taskStatus = task_manager_msgs::TaskStatus::TASK_CONFIGURED;
}

const std::string & TaskDefinition::getStatusString() const {
	return statusString;
}


void TaskDefinition::debug(const char *stemplate,...) const {
	va_list args;
	printf("%s: ",name.c_str());
	va_start(args, stemplate);
	vprintf(stemplate,args);
	va_end(args);
}

void TaskDefinition::doConfigure(const TaskParameters & parameters)
{
    dynamic_reconfigure::ConfigTools::getParameter(parameters,"task_timeout",timeout);

	statusString.clear();
    config = parameters;
	taskStatus = this->configure(parameters);
}

void TaskDefinition::doInitialise(const TaskParameters & parameters)
{
    dynamic_reconfigure::ConfigTools::getParameter(parameters,"task_timeout",timeout);
	statusString.clear();
    config = parameters;
	taskStatus = this->initialise(parameters);
}

void TaskDefinition::doIterate()
{
	statusString.clear();
	taskStatus = this->iterate();
}

void TaskDefinition::doTerminate()
{
	statusString.clear();
	taskStatus = this->terminate();
}

const char * taskStatusToString(TaskIndicator ts)
{
	switch (ts) {
        case task_manager_msgs::TaskStatus::TASK_NEWBORN: return "NEWBORN"; 
		case task_manager_msgs::TaskStatus::TASK_CONFIGURED: return "CONFIGURED";
		case task_manager_msgs::TaskStatus::TASK_INITIALISED: return "INITIALISED";
		case task_manager_msgs::TaskStatus::TASK_RUNNING: return "RUNNING";
		case task_manager_msgs::TaskStatus::TASK_COMPLETED: return "COMPLETED";
		case task_manager_msgs::TaskStatus::TASK_TERMINATED: return "TERMINATED";
		case task_manager_msgs::TaskStatus::TASK_FAILED: return "FAILED";
		case task_manager_msgs::TaskStatus::TASK_INTERRUPTED: return "INTERRUPTED";
		case task_manager_msgs::TaskStatus::TASK_TIMEOUT: return "TIMEOUT";
		case task_manager_msgs::TaskStatus::TASK_CONFIGURATION_FAILED: return "CONFIGURATION FAILED";
		case task_manager_msgs::TaskStatus::TASK_INITIALISATION_FAILED: return "INITIALISATION FAILED";
		default: return "INVALID STATUS";
	}
	return NULL;
}

