#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_msgs/msg/task_description.hpp>
#include <task_manager_msgs/msg/task_config.hpp>
#include "task_manager_lib/TaskParameterDefinition.h"


namespace task_manager_lib {


    class TaskConfig {
        public:
            typedef std::map<std::string,TaskParameterDefinition> TaskConfigMap;
        protected:
            std::string ns; // namespace
            TaskConfigMap definitions;

            class GenVariableUpdater {
                public:
                    virtual void update(const TaskConfig & cfg, const std::string & name) = 0;
                    GenVariableUpdater() {}
                    virtual ~GenVariableUpdater() {}
            };

            template <class ParameterT>
                class VariableUpdater : public GenVariableUpdater {
                    protected:
                        ParameterT & var;
                    public:
                        VariableUpdater(ParameterT & v) : var(v) {}
                        virtual ~VariableUpdater() {}
                        virtual void update(const TaskConfig & cfg, const std::string & name) {
                            var = cfg.get<ParameterT>(name);
                        }
                };

            typedef std::shared_ptr<GenVariableUpdater> VariableUpdaterPtr;

            std::map<std::string,VariableUpdaterPtr> updaters;

        public:
            TaskConfig() : ns("") {
                // define("task_rename","","used to rename a task at runtime [deprecated]",true);
                define("foreground",true,"run this task in foreground or not",true);
                define("task_period",1.0,"default task period for periodic task",true);
                define("task_timeout",-1.0,"if positive, maximum time this task can run for",true);
            }
            virtual ~TaskConfig() {}

            template <class ParameterT> 
                void define(const std::string & name,
                        const ParameterT & val,
                        const std::string & description,
                        bool read_only=true) {
                    definitions[name]=TaskParameterDefinition(name,val,description,read_only);
                }

            template <class ParameterT> 
                void define(const std::string & name,
                        const ParameterT & val,
                        const std::string & description,
                        bool read_only,ParameterT & var) {
                    definitions[name]=TaskParameterDefinition(name,val,description,read_only);
                    updaters[name] = VariableUpdaterPtr(new VariableUpdater<ParameterT>(var));
                }

            void setNameSpace(const std::string & name) {
                ns = name;
            }
            const std::string & getNameSpace() const {
                return ns;
            }



            bool declareParameters(rclcpp::Node::SharedPtr node);
            bool undeclareParameters(rclcpp::Node::SharedPtr node);
            void updateParameters(rclcpp::Node::SharedPtr node);
            void publishParameters(rclcpp::Node::SharedPtr node);

            std::vector<rcl_interfaces::msg::ParameterDescriptor> getParameterDescriptions() const;

            void updateLinkedVariables() ;
            void loadConfig(const task_manager_msgs::msg::TaskConfig & cfg) ;

            void loadConfig(const TaskConfig & cfg) ;

            void loadConfig(const std::vector<rclcpp::Parameter> & cfg,const std::string & prefix) ;

            void loadConfig(const std::vector<rcl_interfaces::msg::Parameter> & cfg, const std::string & prefix) ;

            bool getDefinition(const std::string & name,TaskParameterDefinition & def);

            template <class ParameterT> 
                ParameterT get(const std::string & name) const {
                    TaskConfigMap::const_iterator it = definitions.find(name);
                    if (it == definitions.end()) {
                        RCLCPP_WARN(rclcpp::get_logger("TaskConfig"),"No parameter '%s' in config",name.c_str());
                        return ParameterT();
                    }
                    return it->second.get<ParameterT>();
                }

            bool exportToMessage(std::vector<rcl_interfaces::msg::Parameter> & plist) const ;
            void printConfig() const ;
    };


    typedef std::shared_ptr<TaskConfig> TaskConfigPtr;
    typedef std::shared_ptr<TaskConfig const> TaskConfigConstPtr;



}

#endif // TASK_CONFIG_H
