
#include <dynamic_reconfigure/config_tools.h>
#include "task_manager_lib/TaskDefinition.h"
using namespace task_manager_lib;

void TaskDefinitionBase::setName(const std::string & n) {
	name = n;
}

void TaskDefinitionBase::setTaskId(unsigned int id) {
	taskId = id;
}

void TaskInstanceBase::setRuntimeId(unsigned int id) {
	runId = id;
}

unsigned int TaskDefinitionBase::getTaskId() const {
    return taskId;
}

unsigned int TaskInstanceBase::getRuntimeId() const {
    return runId;
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


TaskIndicator TaskDefinitionBase::getStatus() const {
	return taskStatus;
}

void TaskDefinitionBase::setStatus(const TaskIndicator & ti) {
	taskStatus = ti;
}

task_manager_msgs::TaskDescription TaskDefinitionBase::getDescription() const {
    task_manager_msgs::TaskDescription td;
    td.name = this->getName();
    td.description = this->getHelp();
    td.periodic = this->isPeriodic();
    td.config = this->getParameterDescription();
    return td;
}

void TaskDefinitionBase::doConfigure(unsigned int id, const TaskParameters & parameters)
{
    taskId = id;
    // printf("Configure %s: default values\n",this->getName().c_str());
    // config.print(stdout);
    // printf("Configure %s: proposed values\n",this->getName().c_str());
    // parameters.print(stdout);
    // printf("Configure %s: after update\n",this->getName().c_str());
    // config.print(stdout);

	statusString.clear();
	taskStatus = this->configure(parameters);
}


const std::string & TaskDefinitionBase::getStatusString() const {
	return statusString;
}

void TaskDefinitionBase::setStatusString(const std::string & s) {
    statusString = s;
}


void TaskDefinitionBase::debug(const char *stemplate,...) const {
	va_list args;
    char buffer[1024];
	va_start(args, stemplate);
	vsnprintf(buffer,1023, stemplate,args);
	va_end(args);
    buffer[1023]=0;
	ROS_INFO("%s: %s",this->getName().c_str(),buffer);
}

TaskIndicator TaskInstanceBase::getStatus() const {
	return taskStatus;
}

void TaskInstanceBase::setStatus(const TaskIndicator & ti) {
	taskStatus = ti;
}

TaskDefinitionPtr TaskInstanceBase::getDefinition() {
    return definition;
}

task_manager_msgs::TaskStatus TaskInstanceBase::getRosStatus() const {
    task_manager_msgs::TaskStatus st;
    st.name = this->getName();
    st.status = this->getStatus();
    st.status_string = this->getStatusString();
    return st;
}

const std::string & TaskInstanceBase::getStatusString() const {
	return statusString;
}

void TaskInstanceBase::setStatusString(const std::string & s) {
    statusString = s;
}

double TaskInstanceBase::getTimeout() const {
	return timeout;
}


void TaskInstanceBase::debug(const char *stemplate,...) const {
	va_list args;
    char buffer[1024];
	va_start(args, stemplate);
	vsnprintf(buffer,1023, stemplate,args);
	va_end(args);
    buffer[1023]=0;
	ROS_INFO("%s: %s",this->getName().c_str(),buffer);
}

bool TaskInstanceBase::isAnInstanceOf(const TaskDefinitionBase & def) {
    return this->getDefinition()->getTaskId() == def.getTaskId();
}

bool TaskInstanceBase::isAnInstanceOf(TaskDefinitionConstPtr def) {
    return this->getDefinition()->getTaskId() == def->getTaskId();
}

void TaskInstanceBase::doInitialise(unsigned int runtimeId, const TaskParameters & parameters)
{
    boost::shared_lock<boost::shared_mutex> guard(env_gen->environment_mutex);
    runId = runtimeId;
    // printf("Initialise %s: after update\n",this->getName().c_str());
    // config.print(stdout);
    parameters.getParameter("task_timeout",timeout);
	statusString.clear();
    this->parseParameters(parameters);
	taskStatus = this->initialise();
}

void TaskInstanceBase::doIterate()
{
	statusString.clear();
    // TODO: improve ways to specify what requires locking
    if (this->isPeriodic()) {
        boost::shared_lock<boost::shared_mutex> guard(env_gen->environment_mutex);
        taskStatus = this->iterate();
    } else {
        taskStatus = this->iterate();
    }
}

void TaskInstanceBase::doTerminate()
{
    boost::shared_lock<boost::shared_mutex> guard(env_gen->environment_mutex);
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

