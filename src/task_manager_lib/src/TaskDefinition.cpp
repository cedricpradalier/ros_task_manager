
// #include <dynamic_reconfigure/config_tools.h>
#include "task_manager_lib/TaskDefinition.h"
using namespace task_manager_lib;
using std::placeholders::_1;


bool TaskConfig::declareParameters(rclcpp::Node * node) {
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        node->declare_parameter(it->first,it->second->getDefaultValue(),it->second->getDescription());
    }
    return true;
}


void TaskConfig::updateParameters(rclcpp::Node * node) {
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        if (!node->get_parameter(it->first,it->second->getValue())) {
            it->second->setDefaultValue();
        }
    }
}

void TaskConfig::publishParameters(rclcpp::Node * node) {
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        rclcpp::Parameter p(it->first,it->second->getValue());
        node->set_parameter(p);
    }
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> TaskConfig::getParameterDescriptions() {
    std::vector<rcl_interfaces::msg::ParameterDescriptor> res;
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        res.push_back(it->second->getDescription());
    }
    return res;
}





void TaskDefinitionBase::setName(const std::string & n) {
	name = n;
}

void TaskDefinitionBase::setTaskId(unsigned int id) {
	taskId = id;
}

unsigned int TaskDefinitionBase::getTaskId() const {
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
    RCLCPP_INFO(this->get_logger(),"%s: %s",this->getName().c_str(),buffer);
}

#if 1
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
	RCLCPP_INFO(this->get_logger(),"%s: %s",this->getName().c_str(),buffer);
}

bool TaskInstanceBase::isAnInstanceOf(const TaskDefinitionBase & def) {
    return this->getDefinition()->getTaskId() == def.getTaskId();
}

bool TaskInstanceBase::isAnInstanceOf(TaskDefinitionConstPtr def) {
    return this->getDefinition()->getTaskId() == def->getTaskId();
}

void TaskInstanceBase::doInitialise(unsigned int runtimeId, const task_manager_msgs::msg::TaskConfig & parameters)
{
    std::unique_lock<std::mutex> guard(env_gen->environment_mutex);
    runId = runtimeId;
    // printf("Initialise %s: after update\n",this->getName().c_str());
    // config.print(stdout);
    cfg_gen->loadConfig(parameters);
    cfg_gen->publishParameters(this);
    timeout = cfg_gen->task_timeout.get<double>();
	statusString.clear();

    
    taskStatus = this->initialise();

}

void TaskInstanceBase::updateParameters() {
    cfg_gen->updateParameters(this);
}

void TaskInstanceBase::doIterate()
{
    cfg_gen->updateParameters(this);
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
}

void TaskInstanceBase::doTerminate()
{
    cfg_gen->updateParameters(this);
    std::unique_lock<std::mutex> guard(env_gen->environment_mutex);
	statusString.clear();
    TaskIndicator ti = this->terminate();
    if (ti == task_manager_msgs::msg::TaskStatus::TASK_TERMINATED) {
        taskStatus |= task_manager_msgs::msg::TaskStatus::TASK_TERMINATED; 
    } else {
        taskStatus = ti | task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
    }

    std::vector<std::string> prefixes;
    rcl_interfaces::msg::ListParametersResult lparams = this->list_parameters(prefixes,0);
    for (size_t i=0;i<lparams.names.size();i++) {
        this->undeclare_parameter(lparams.names[i]);
    }

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
#endif

