#ifndef TASK_INSTANCE_H
#define TASK_INSTANCE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include <thread>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_msgs/msg/task_status.hpp>
#include <task_manager_msgs/msg/task_description.hpp>
#include <task_manager_msgs/msg/task_config.hpp>

#include "task_manager_lib/TaskEnvironment.h"
#include "task_manager_lib/TaskConfig.h"
#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {

    /**
     *
     * Mother class of all tasks. Contains all the generic tools to define a task.
     * Must be inherited. Such a task can be periodic or aperiodic
     * 
     * */
    class TaskInstanceBase : public std::enable_shared_from_this<TaskInstanceBase>
    {
        protected:
            rclcpp::Node::SharedPtr node;
            rclcpp::Node::SharedPtr getNode() {return node;}
            rclcpp::Node::ConstSharedPtr getNode() const {return node;}
            TaskDefinitionPtr definition;
            /**
             * Storage for some task status string, if required. Can only be set
             * with setStatusString
             * */
            std::string statusString;

            /**
             * Storage for the current status of the task. Automatically updated
             * from the output of the configure, initialise, iterate and terminate
             * functions
             * */
            TaskIndicator taskStatus;

            /**
             * Timeout value for this cycle
             * */
            double timeout;

            /**
             * Id of the task, set by the scheduler when initializing the
             * instance
             * */
            unsigned int runId;

            TaskEnvironmentPtr env_gen;
            TaskConfigPtr cfg_gen;

            bool readyForReconfigure;
            // Setup a dynamic reconfigure server that just update all the
            // config. To be updated 
            virtual void reconfigure() {
            }



            // Return the parameters as read from the parameter server. Returns the
            // default parameters otherwise.
            void updateParameters() ;

        public:
            // All the class below are intended for generic use

            // Default constructor:
            // task parameters.
            TaskInstanceBase(TaskDefinitionPtr def, TaskEnvironmentPtr ev, TaskConfigPtr cfg) :
                node(ev->getNode()), definition(def), taskStatus(task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED), 
                timeout(-1.0), runId(-1), env_gen(ev), cfg_gen(cfg) {
                    if (!cfg_gen) {
                        cfg_gen.reset(new TaskConfig);
                    }
                    cfg_gen->setNameSpace(def->getInstanceName()+".");
                    readyForReconfigure = false;

                }
            virtual ~TaskInstanceBase() {
                // printf("Delete task instance'%s'\n",name.c_str());
                // fflush(stdout);
            }


            // Set the task runtime id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual unsigned int getRuntimeId() const;
            // Set the task runtime id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual TaskDefinitionPtr getDefinition();
            virtual TaskDefinitionConstPtr getDefinition() const;

            // Get the status indicator 
            // Has to be virtual because it is overloaded by the dynamic class proxy.
            virtual TaskIndicator getStatus() const;
            // Update the task status
            virtual void setStatus(const TaskIndicator & ti);
            // Get the status string 
            // Has to be virtual because it is overloaded by the dynamic class proxy.
            virtual const std::string & getStatusString() const;
            // Update the task status string
            virtual void setStatusString(const std::string & s); 

            // Report the task timeout
            // Has to be virtual because it is overloaded by the dynamic class proxy.
            virtual double getTimeout() const;


            TaskEnvironmentPtr getEnvironment() {return env_gen;}
            TaskConfigPtr getConfig() {return cfg_gen;}
            TaskConfigConstPtr getConfig() const {return cfg_gen;}
        public:
            const std::string & getName() const {
                return definition->getName();
            }

            bool isPeriodic() const {
                return definition->isPeriodic();
            }

            // All the functions below are intended for the TaskScheduler.
            // Set the task runtime id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual void setRuntimeId(unsigned int id);

            // Test if a class is an instance of def
            bool isAnInstanceOf(const TaskDefinitionBase & def) const;
            bool isAnInstanceOf(TaskDefinitionConstPtr def) const;

            // Call the virtual initialise function, but prepare the class before
            // hand.
            void doInitialise(unsigned int runtimeId, const TaskConfig & parameters);

            // Call the virtual iterate function, but prepare the class before
            // hand.
            void doIterate();

            // Call the virtual terminate function, but prepare the class before
            // hand.
            void doTerminate();

            // Output a debug string, prefixed by the task name
            void debug(const char *stemplate,...) const; 

            // Get the status as a message ready to be published over ROS
            task_manager_msgs::msg::TaskStatus getRosStatus() const;

        public:
            // The functions below are virtual pure and must be implemented by the
            // specific task by linking in the type generated from the .cfg file. 
            // See the TaskDefinitionWithConfig class for details.
            rcl_interfaces::msg::SetParametersResult
                reconfigure_callback(const std::vector<rclcpp::Parameter> & parameters)
                {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    cfg_gen->loadConfig(parameters,cfg_gen->getNameSpace()); 
                    this->reconfigure();
                    return result;
                }

            bool isReadyForReconfigure() const {
                return readyForReconfigure;
            }

        protected:
            // Set of functions only useful for derived classes
            friend class DynamicTask;

        protected:
            // Set of functions that must be implemented by any inheriting class

            // Initialise is called once every time the task is launched
            virtual TaskIndicator initialise() {
                RCLCPP_INFO(node->get_logger(),"Initialising task %s: default function",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_INITIALISED;
            }

            // iterate is called only once for non periodic tasks. It is called
            // iteratively with period 'task_period' for periodic class. 
            virtual TaskIndicator iterate() {
                RCLCPP_INFO(node->get_logger(),"Task %s: default iteration",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_COMPLETED;
            }

            // Terminate is called once when the task is completed, cancelled or
            // interrupted.
            virtual TaskIndicator terminate() {
                RCLCPP_INFO(node->get_logger(),"Terminating task %s: default function",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
            }

    };



    // Templated class specialising some of the virtual functions of a
    // TaskDefinition based on the data available in a XXXConfig class generated
    // from a .cfg file. This is still a virtual pure class.
    template <class CFG, class ENV>
        class TaskInstance : public TaskInstanceBase {
            public:
                typedef TaskInstance<CFG,ENV> Parent;
            protected:
                std::shared_ptr<ENV> env;
                std::shared_ptr<CFG> cfg;

                std::shared_ptr<ENV> castEnvironment() {
                    std::shared_ptr<ENV> e = std::dynamic_pointer_cast<ENV,TaskEnvironment>(env_gen);
                    assert(e);
                    return e;
                }
                std::shared_ptr<CFG> castConfig() {
                    std::shared_ptr<CFG> e = std::dynamic_pointer_cast<CFG,TaskConfig>(cfg_gen);
                    assert(e);
                    return e;
                }
            public:
                // Same constructor as the normal TaskDefinition
                TaskInstance(TaskDefinitionPtr def, TaskEnvironmentPtr ev) 
                    : TaskInstanceBase(def,ev,std::shared_ptr<CFG>(new CFG)) {
                        env = castEnvironment();
                        cfg = castConfig();
                    }
                virtual ~TaskInstance() {}

                virtual TaskIndicator initialise() 
                {
                    return task_manager_msgs::msg::TaskStatus::TASK_INITIALISED;
                }

                virtual TaskIndicator terminate()
                {
                    return task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
                }

                template <class SPECIALIZED>
                    std::shared_ptr<SPECIALIZED> castDefinition() {
                        std::shared_ptr<SPECIALIZED> d = std::dynamic_pointer_cast<SPECIALIZED,TaskDefinitionBase>(definition);
                        assert(d);
                        return d;
                    }

        };


}

#endif // TASK_INSTANCE_H
