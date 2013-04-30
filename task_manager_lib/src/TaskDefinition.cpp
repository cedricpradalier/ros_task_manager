
#include <dynamic_reconfigure/config_tools.h>
#include "task_manager_lib/TaskDefinition.h"
using namespace task_manager_lib;

void TaskDefinition::setName(const std::string & n) {
	name = n;
}

void TaskDefinition::setTaskId(unsigned int id) {
	taskId = id;
}

void TaskDefinition::setRuntimeId(unsigned int id) {
	runId = id;
}

unsigned int TaskDefinition::getTaskId() const {
    return taskId;
}

unsigned int TaskDefinition::getRuntimeId() const {
    return runId;
}

const std::string & TaskDefinition::getName() const {
	return name;
}

const std::string & TaskDefinition::getHelp() const {
	return help;
}

const TaskParameters & TaskDefinition::getConfig() const {
	return config;
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
    st.name = this->getName();
    st.status = this->getStatus();
    st.status_string = this->getStatusString();
    st.plist = this->getConfig();
    return st;
}

task_manager_msgs::TaskDescription TaskDefinition::getDescription() const {
    task_manager_msgs::TaskDescription td;
    td.name = this->getName();
    td.description = this->getHelp();
    td.periodic = this->isPeriodic();
    td.timeout_s = this->getTimeout();
    td.config = this->getParameterDescription();
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
    char buffer[1024];
	va_start(args, stemplate);
	vsnprintf(buffer,1023, stemplate,args);
	va_end(args);
    buffer[1023]=0;
	ROS_INFO("%s: %s",this->getName().c_str(),buffer);
}

bool TaskDefinition::isAnInstanceOf(const TaskDefinition & def) {
    return this->getTaskId() == def.getTaskId();
}

bool TaskDefinition::isAnInstanceOf(const boost::shared_ptr<TaskDefinition> & def) {
    return this->getTaskId() == def->getTaskId();
}

void TaskDefinition::doConfigure(unsigned int id, const TaskParameters & parameters)
{
    taskId = id;
    config = this->getDefaultParameters();
    // printf("Configure %s: default values\n",this->getName().c_str());
    // config.print(stdout);
    // printf("Configure %s: proposed values\n",this->getName().c_str());
    // parameters.print(stdout);
    config.update(parameters);
    // printf("Configure %s: after update\n",this->getName().c_str());
    // config.print(stdout);

    parameters.getParameter("task_timeout",timeout);


	statusString.clear();
	taskStatus = this->configure(parameters);
}

void TaskDefinition::doInitialise(unsigned int runtimeId, const TaskParameters & parameters)
{
    runId = runtimeId;
    config.update(parameters);
    // printf("Initialise %s: after update\n",this->getName().c_str());
    // config.print(stdout);
    parameters.getParameter("task_timeout",timeout);
	statusString.clear();
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
    TaskIndicator ti = this->terminate();
    if (ti == task_manager_msgs::TaskStatus::TASK_TERMINATED) {
        taskStatus |= task_manager_msgs::TaskStatus::TASK_TERMINATED; 
    } else {
        taskStatus = ti | task_manager_msgs::TaskStatus::TASK_TERMINATED;
    }
}

const char * task_manager_lib::taskStatusToString(TaskIndicator ts)
{
    unsigned int te = task_manager_msgs::TaskStatus::TASK_TERMINATED;
    if (ts == te) {
		return "TERMINATED";
    } else if (ts & te) {
        // Terminated
        ts = ts & (~te);
        switch (ts) {
            case task_manager_msgs::TaskStatus::TASK_COMPLETED: return "TERMINATED:COMPLETED";
            case task_manager_msgs::TaskStatus::TASK_FAILED: return "TERMINATED:FAILED";
            case task_manager_msgs::TaskStatus::TASK_INTERRUPTED: return "TERMINATED:INTERRUPTED";
            case task_manager_msgs::TaskStatus::TASK_TIMEOUT: return "TERMINATED:TIMEOUT";
            case task_manager_msgs::TaskStatus::TASK_CONFIGURATION_FAILED: return "TERMINATED:CONFIGURATION FAILED";
            case task_manager_msgs::TaskStatus::TASK_INITIALISATION_FAILED: return "TERMINATED:INITIALISATION FAILED";
            default: return "INVALID STATUS";
        }
    } else {
        // Not terminated
        switch (ts) {
            case task_manager_msgs::TaskStatus::TASK_NEWBORN: return "NEWBORN"; 
            case task_manager_msgs::TaskStatus::TASK_CONFIGURED: return "CONFIGURED";
            case task_manager_msgs::TaskStatus::TASK_INITIALISED: return "INITIALISED";
            case task_manager_msgs::TaskStatus::TASK_RUNNING: return "RUNNING";
            case task_manager_msgs::TaskStatus::TASK_COMPLETED: return "COMPLETED";
            case task_manager_msgs::TaskStatus::TASK_FAILED: return "FAILED";
            case task_manager_msgs::TaskStatus::TASK_TIMEOUT: return "TIMEOUT";
            case task_manager_msgs::TaskStatus::TASK_CONFIGURATION_FAILED: return "CONFIGURATION FAILED";
            case task_manager_msgs::TaskStatus::TASK_INITIALISATION_FAILED: return "INITIALISATION FAILED";
            default: return "INVALID STATUS";
        }
    }
	return NULL;
}

