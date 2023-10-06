#ifndef TASK_PARAMETER_DEFINITION_H
#define TASK_PARAMETER_DEFINITION_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include <thread>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_msgs/msg/task_description.hpp>
#include <task_manager_msgs/msg/task_config.hpp>


namespace task_manager_lib {

    class TaskParameterDefinition {
        protected:
            rclcpp::ParameterValue value;
            rclcpp::ParameterValue defaultValue;
            rcl_interfaces::msg::ParameterDescriptor descriptor;
        public:
            TaskParameterDefinition() {
                descriptor.name = "undefined";
            }

            template <class ParameterT> 
                TaskParameterDefinition(const std::string & name,
                        const ParameterT & val,
                        const std::string & description,
                        bool read_only) : value(val), defaultValue(val) {
                    descriptor.name = name;
                    descriptor.type = value.get_type();
                    descriptor.description = description;
                    descriptor.read_only = read_only;
                    descriptor.dynamic_typing = true; // from galactic, necessary to allow undeclaring
                }
            virtual ~TaskParameterDefinition(){}
            
            template <class ParameterT> 
                ParameterT get() const {
                    return value.get<ParameterT>();
                }

            void setDefaultValue() {
                value = defaultValue;
            }

            bool readOnly() const {
                return descriptor.read_only;
            }

            const rclcpp::ParameterValue & getValue() const {
                return value;
            }
            void setValue(const rclcpp::ParameterValue & val) {
                value = val;
            }
            void setValue(const rcl_interfaces::msg::ParameterValue & val) {
                value = rclcpp::ParameterValue(val);
            }
            template <class ParameterT>
                void setValue(const ParameterT & val) {
                    value = rclcpp::ParameterValue(val);
                }

            rclcpp::ParameterValue & getValue() {
                return value;
            }
            const rclcpp::ParameterValue & getDefaultValue() const {
                return defaultValue;
            }
            const rcl_interfaces::msg::ParameterDescriptor getDescription() const {
                return descriptor;
            }
    };

}

#endif // TASK_PARAMETER_DEFINITION_H
