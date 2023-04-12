
#include "task_manager_lib/TaskConfig.h"
using namespace task_manager_lib;
using std::placeholders::_1;


bool TaskConfig::declareParameters(rclcpp::Node::SharedPtr node) {
    // printConfig();
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        if (!it->second.readOnly()) {
            // RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Declare %s",(ns+it->first).c_str());
            node->declare_parameter(ns+it->first,it->second.getValue(),it->second.getDescription());
        }
    }
    return true;
}


bool TaskConfig::undeclareParameters(rclcpp::Node::SharedPtr node) {
    // printConfig();
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        if (!it->second.readOnly()) {
            try { 
                // RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Undeclare %s",(ns+it->first).c_str());
                node->undeclare_parameter(ns+it->first);
            } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
                RCLCPP_WARN(rclcpp::get_logger("TaskConfig"),"undeclared exception while undeclaring '%s'",(ns+it->first).c_str());
            }
        }
    }
    return true;
}


void TaskConfig::updateParameters(rclcpp::Node::SharedPtr node) {
    for (TaskConfigMap::iterator it=definitions.begin();it!=definitions.end();it++) {
        if (it->second.readOnly()) {
            // This cannot be updated
            continue;
        }
        if (!node->get_parameter(ns+it->first,it->second.getValue())) {
            it->second.setDefaultValue();
        }
        //rcl_interfaces::msg::ParameterValue pv = it->second.getValue().to_value_msg();
        //RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Get %s: %d %ld %f %s",(ns+it->first).c_str(), pv.bool_value,pv.integer_value,pv.double_value);
    }
}

void TaskConfig::publishParameters(rclcpp::Node::SharedPtr node) {
    std::vector<rclcpp::Parameter> parameters;
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        if (it->second.readOnly()) {
            // This cannot be updated
            continue;
        }
        rclcpp::Parameter p(ns+it->first,it->second.getValue());
        parameters.push_back(p);
        //rcl_interfaces::msg::ParameterValue pv = it->second.getValue().to_value_msg();
        //RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Set %s: %d %ld %f %s",(ns+it->first).c_str(), pv.bool_value,pv.integer_value,pv.double_value);
    }
    node->set_parameters_atomically(parameters);
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> TaskConfig::getParameterDescriptions() const {
    std::vector<rcl_interfaces::msg::ParameterDescriptor> res;
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        res.push_back(it->second.getDescription());
    }
    return res;
}


bool TaskConfig::exportToMessage(std::vector<rcl_interfaces::msg::Parameter> & plist) const {
    plist.clear();
    for (TaskConfigMap::const_iterator it=definitions.begin();it!=definitions.end();it++) {
        rcl_interfaces::msg::Parameter P;
        P.name = ns+it->first;
        P.value = it->second.getValue().to_value_msg();
        plist.push_back(P);
    }
    return true;
}

void TaskConfig::printConfig() const {
    for (TaskConfigMap::const_iterator mit=definitions.begin();mit!=definitions.end();mit++) {
        rcl_interfaces::msg::ParameterValue pv = mit->second.getValue().to_value_msg();
        RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"%s: %d %ld %f %s [%s]",mit->first.c_str(),
                pv.bool_value,pv.integer_value,pv.double_value,
                mit->second.getDescription().description.c_str(),
                mit->second.readOnly()?"RO":"RW");
    }
}

void TaskConfig::updateLinkedVariables() {
    for (std::map<std::string,VariableUpdaterPtr>::iterator it=updaters.begin();it!=updaters.end();it++) {
        it->second->update(*this,it->first);
    }
}

void TaskConfig::loadConfig(const task_manager_msgs::msg::TaskConfig & cfg) {
    // RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Load config msgTC %d param", int(cfg.plist.size()));
    for (size_t i=0;i<cfg.plist.size();i++) {
        std::string name = cfg.plist[i].name;
        TaskConfigMap::iterator it = definitions.find(name);
        if (it == definitions.end()) {
            continue;
        }
        it->second.setValue(cfg.plist[i].value);
    }
    updateLinkedVariables();
}

void TaskConfig::loadConfig(const TaskConfig & cfg) {
    // RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Load config TC %d", int(cfg.definitions.size()));
    // cfg.printConfig();
    for (TaskConfigMap::const_iterator it=cfg.definitions.begin();it!=cfg.definitions.end();it++) {
        definitions[it->first] = it->second;
    }
    updateLinkedVariables();
}

void TaskConfig::loadConfig(const std::vector<rclcpp::Parameter> & cfg,const std::string & prefix) {
    for (std::vector<rclcpp::Parameter>::const_iterator it=cfg.begin();it!=cfg.end();it++) {
        if (it->get_name().rfind(prefix,0)!=0) {
            // ignoring parameter, this is not for this task
            continue;
        }
        assert(it->get_name().size() >= prefix.size());
        std::string name = it->get_name().substr(prefix.size(),std::string::npos);
        TaskConfigMap::iterator dit = definitions.find(name);
        if (dit == definitions.end()) {
            continue;
        }
        dit->second.setValue(it->get_parameter_value());
    }
    updateLinkedVariables();
}

void TaskConfig::loadConfig(const std::vector<rcl_interfaces::msg::Parameter> & cfg, const std::string & prefix) {
    // RCLCPP_INFO(rclcpp::get_logger("TaskConfig"),"Load config PL %d param",int(cfg.size()));
    for (std::vector<rcl_interfaces::msg::Parameter>::const_iterator it=cfg.begin();it!=cfg.end();it++) {
        std::string name = it->name.substr(prefix.size(),std::string::npos);
        TaskConfigMap::iterator dit = definitions.find(name);
        if (dit == definitions.end()) {
            // RCLCPP_WARN(rclcpp::get_logger("TaskConfig"),"Ignoring parameter '%s': not found in the following list:",name.c_str());
            // printConfig();
            continue;
        }
        dit->second.setValue(rclcpp::ParameterValue(it->value));
    }
    updateLinkedVariables();
}

bool TaskConfig::getDefinition(const std::string & name,TaskParameterDefinition & def) {
    TaskConfigMap::iterator it = definitions.find(name);
    if (it == definitions.end()) {
        return false;
    }
    def = it->second;
    return true;
}



