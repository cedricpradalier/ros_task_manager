
#include "task_manager_lib/TaskInstance.h"
using namespace task_manager_lib;
using std::placeholders::_1;



unsigned int TaskInstanceBase::getRuntimeId() const {
    return runId;
}

void TaskInstanceBase::setRuntimeId(unsigned int id) {
	runId = id;
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

TaskDefinitionConstPtr TaskInstanceBase::getDefinition() const {
    return definition;
}

task_manager_msgs::msg::TaskStatus TaskInstanceBase::getRosStatus() const {
    task_manager_msgs::msg::TaskStatus st;
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
	RCLCPP_INFO(node->get_logger(),"%s: %s",this->getName().c_str(),buffer);
}

bool TaskInstanceBase::isAnInstanceOf(const TaskDefinitionBase & def) const {
    // RCLCPP_INFO(node->get_logger(),  " %d isAnInstanceOf task %d: %d",this->getDefinition()->getTaskId(), def.getTaskId(), int(this->getDefinition()->getTaskId() == def.getTaskId()));
    return this->getDefinition()->getTaskId() == def.getTaskId();
}

bool TaskInstanceBase::isAnInstanceOf(TaskDefinitionConstPtr def) const {
    // RCLCPP_INFO(node->get_logger(),  " %d isAnInstanceOf task %d: %d",this->getDefinition()->getTaskId(), def->getTaskId(), int(this->getDefinition()->getTaskId() == def->getTaskId()));
    return this->getDefinition()->getTaskId() == def->getTaskId();
}

void TaskInstanceBase::doInitialise(unsigned int runtimeId, const task_manager_lib::TaskConfig & parameters)
{
    std::unique_lock<std::mutex> guard(env_gen->environment_mutex);
    runId = runtimeId;
    // printf("Initialise %s: after update\n",this->getName().c_str());
    // config.print(stdout);
    // RCLCPP_INFO(node->get_logger(),"doInitialise Task Config");
    // parameters.printConfig();
    cfg_gen->loadConfig(parameters);
    // RCLCPP_INFO(node->get_logger(),"After loading Task Config");
    // cfg_gen->printConfig();
    cfg_gen->declareParameters(node);
    cfg_gen->publishParameters(node); // Not relevant for R/O params
    timeout = cfg_gen->get<double>("task_timeout");
	statusString.clear();

    readyForReconfigure = true;

    
    taskStatus = this->initialise();

    switch (taskStatus) {
        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISED: 
        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISATION_FAILED:
            break;

        case task_manager_msgs::msg::TaskStatus::TASK_NEWBORN: 
        case task_manager_msgs::msg::TaskStatus::TASK_TERMINATED: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED: 
        case task_manager_msgs::msg::TaskStatus::TASK_RUNNING: 
        case task_manager_msgs::msg::TaskStatus::TASK_COMPLETED: 
        case task_manager_msgs::msg::TaskStatus::TASK_FAILED: 
        case task_manager_msgs::msg::TaskStatus::TASK_TIMEOUT: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURATION_FAILED:
        default: 
            RCLCPP_WARN(node->get_logger(), "Task %s: initialise returned a weird status %s",
                    getName().c_str(),taskStatusToString(taskStatus));
            break;
    }
}

void TaskInstanceBase::updateParameters() {
    cfg_gen->updateParameters(node);
}

void TaskInstanceBase::doIterate()
{
    cfg_gen->updateParameters(node);
    // RCLCPP_INFO(node->get_logger(),"Iterate: After update Task Config");
    // cfg_gen->printConfig();
	statusString.clear();
    // TODO: improve ways to specify what requires locking
    if (this->isPeriodic()) {
        std::unique_lock<std::mutex> guard(env_gen->environment_mutex);
        taskStatus = this->iterate();
    } else {
        // Forcing status to RUNNING to avoid task not appearing in task client
        taskStatus = task_manager_msgs::msg::TaskStatus::TASK_RUNNING;
        taskStatus = this->iterate();
    }
    switch (taskStatus) {
        case task_manager_msgs::msg::TaskStatus::TASK_RUNNING: 
        case task_manager_msgs::msg::TaskStatus::TASK_COMPLETED: 
        case task_manager_msgs::msg::TaskStatus::TASK_FAILED: 
            break;

        case task_manager_msgs::msg::TaskStatus::TASK_TERMINATED: 
        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISATION_FAILED:
        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISED: 
        case task_manager_msgs::msg::TaskStatus::TASK_NEWBORN: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED: 
        case task_manager_msgs::msg::TaskStatus::TASK_TIMEOUT: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURATION_FAILED:
        default: 
            RCLCPP_WARN(node->get_logger(), "Task %s: iterate returned a weird status %s",
                    getName().c_str(),taskStatusToString(taskStatus));
            break;
    }
}

void TaskInstanceBase::doTerminate()
{
    cfg_gen->updateParameters(node);
    std::unique_lock<std::mutex> guard(env_gen->environment_mutex);
	statusString.clear();
    TaskIndicator ti = this->terminate();
    switch (ti) {
        case task_manager_msgs::msg::TaskStatus::TASK_TERMINATED: 
        case task_manager_msgs::msg::TaskStatus::TASK_FAILED: 
            break;

        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISATION_FAILED:
        case task_manager_msgs::msg::TaskStatus::TASK_INITIALISED: 
        case task_manager_msgs::msg::TaskStatus::TASK_NEWBORN: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED: 
        case task_manager_msgs::msg::TaskStatus::TASK_RUNNING: 
        case task_manager_msgs::msg::TaskStatus::TASK_COMPLETED: 
        case task_manager_msgs::msg::TaskStatus::TASK_TIMEOUT: 
        case task_manager_msgs::msg::TaskStatus::TASK_CONFIGURATION_FAILED:
        default: 
            RCLCPP_WARN(node->get_logger(), "Task %s: terminate returned a weird status %s",
                    getName().c_str(),taskStatusToString(taskStatus));
            break;
    }
    if (ti == task_manager_msgs::msg::TaskStatus::TASK_TERMINATED) {
        taskStatus |= task_manager_msgs::msg::TaskStatus::TASK_TERMINATED; 
    } else {
        taskStatus = ti | task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
    }
    cfg_gen->undeclareParameters(node);

}

