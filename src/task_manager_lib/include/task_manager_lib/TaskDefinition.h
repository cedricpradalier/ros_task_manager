#ifndef TASK_DEFINITION_H
#define TASK_DEFINITION_H

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


namespace task_manager_lib {
    class DynamicTask;

    // Enum defined in TaskStatus msg
    typedef unsigned int TaskIndicator;

    typedef task_manager_msgs::msg::TaskStatus TaskStatus;

    class TaskInstanceBase;
    typedef std::shared_ptr<TaskInstanceBase> TaskInstancePtr;
    typedef std::shared_ptr<TaskInstanceBase const> TaskInstanceConstPtr;
    
    

    /**
     * Basic function to build the string representation of one of the status above
     * */
    extern
        const char * taskStatusToString(TaskIndicator ts);

    /**
     * Local exception type for invalid paramters in task argument.
     * Can be thrown by Configure and Initialise member functions
     * */
    struct InvalidParameter : public std::exception {
        std::string text;
        InvalidParameter(const std::string & txt) : text("Invalid Parameter: ") {
            text += txt;
        }
        virtual ~InvalidParameter() throw () {}
        virtual const char * what() const throw () {
            return text.c_str();
        }
    };


    /**
     *
     * Mother class of all tasks. Contains all the generic tools to define a task.
     * Must be inherited. Such a task can be periodic or aperiodic
     * 
     * */
    class TaskDefinitionBase : public std::enable_shared_from_this<TaskDefinitionBase>
    {
        protected:
            rclcpp::Node::SharedPtr node;
            rclcpp::Node::SharedPtr getNode() {return node;}
            /**
             * Task name, for display and also used to find the task in the
             * scheduler 
             * */
            std::string name;
            /**
             * Help string that the task can return on request. For display only.
             * */
            std::string help;
            /**
             * Flag indicating if the task is periodic (iterate is called at regular
             * frequency) or aperiodic (iterate is called only once, but a
             * monitoring function reports regularly about the task state).
             * */
            bool periodic;

            TaskEnvironmentPtr env_gen;
            TaskConfigPtr cfg_gen;

            /**
             * Id of the task, set by the scheduler in the list of known task.
             * Should be unique for a given name
             * */
            unsigned int taskId;


            unsigned int instanceCounter;
        public:
            // All the class below are intended for generic use

            // Default constructor:
            //
            // tname: the task name
            // thelp: the task description
            // isperiodic: tells if the class will be executed recurringly
            // (isperiodic = true), or if the class will be executed in its own
            // thread and will report its status on its own. 
            // task parameters.
            TaskDefinitionBase(const std::string & tname, const std::string & thelp, 
                    bool isperiodic, TaskEnvironmentPtr ev, TaskConfigPtr cfg) :
                node(ev->getNode()), name(tname), help(thelp), periodic(isperiodic), 
                env_gen(ev), cfg_gen(cfg), taskId(-1) {
                    if (cfg_gen) {
                        cfg_gen->setNameSpace(name+".");
                        // This clutters the node namespace, and there is 
                        // no identified use at this stage.
                        // cfg_gen->declareParameters(node);
                    } else {
                        // Only use case is DynamicTask
                    }
                }
            virtual ~TaskDefinitionBase() {
                // printf("Delete task '%s'\n",name.c_str());
                // fflush(stdout);
            }

            virtual std::string getInstanceName() {
                char counter[128];
                sprintf(counter,"%08d",instanceCounter++);
                return name + counter;
            }


            // Set the task runtime id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual int getTaskId() const;

            // Set the task name . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual void setName(const std::string & n);
            // Get the task name . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual const std::string & getName() const;
            // Get the task description . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual const std::string & getHelp() const;

            // Report if the task is meant to be periodic. See above for details
            // Has to be virtual because it is overloaded by the dynamic class proxy.
            virtual bool isPeriodic() const;

            // Get the task description as a combination of task-specific
            // information and dynamic_reconfigure::ConfigDescription (assuming the
            // task as a config file)
            virtual task_manager_msgs::msg::TaskDescription getDescription() const;

            // Provide an instance of the class (or a derivative of it), with
            // its own internal variables that can be run multiple time. 
            virtual TaskInstancePtr instantiate() = 0; 

            virtual TaskEnvironmentPtr getEnvironment() {return env_gen;}
            virtual TaskConfigPtr getConfig() {return cfg_gen;}
            virtual TaskConfigConstPtr getConfig() const {return cfg_gen;}
        public:
            // All the functions below are intended for the TaskScheduler.
            // Set the task id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual void setTaskId(int id);

            // Output a debut string, prefixed by the task name
            void debug(const char *stemplate,...) const; 

        protected:
            // Set of functions only useful for derived classes
            friend class DynamicTask;

            // Update the description string
            virtual void setHelp(const std::string & h) {help = h;}
        protected:
            // Set of functions that must be implemented by any inheriting class

    };

    typedef std::shared_ptr<TaskDefinitionBase> TaskDefinitionPtr;
    typedef std::shared_ptr<TaskDefinitionBase const> TaskDefinitionConstPtr;


    // Templated class specialising some of the virtual functions of a
    // TaskDefinition based on the data available in a XXXConfig class generated
    // from a .cfg file. This is still a virtual pure class.
    template <class CFG, class ENV, class INSTANCE>
        class TaskDefinition: public TaskDefinitionBase {
            protected:
                typedef TaskDefinition<CFG,ENV,INSTANCE> Parent;
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
                TaskDefinition(const std::string & tname, const std::string & thelp, bool isperiodic, 
                        TaskEnvironmentPtr ev) : TaskDefinitionBase(tname,thelp,isperiodic,ev,std::shared_ptr<CFG>(new CFG)) {
                    env = castEnvironment();
                    cfg = castConfig();
                }
                TaskDefinition(const std::string & tname, const std::string & thelp, bool isperiodic, 
                        TaskEnvironmentPtr ev, std::shared_ptr<CFG> cfg_init) : TaskDefinitionBase(tname,thelp,isperiodic,ev,cfg_init) {
                    env = castEnvironment();
                    cfg = cfg_init;
                }
                virtual ~TaskDefinition() {}


                virtual TaskInstancePtr instantiate() {
                    return TaskInstancePtr(new INSTANCE(shared_from_this(),env_gen));
                }


        };

    // Function type for the TaskFactoryObject function that will be inserted into
    // each class to be used as a dynamic class (i.e. a .so library).
    //
    // Note that a class inherited from TaskDefinition and meant to be used with
    // the dynamic loading mecanism must have a constructor with the following
    // profile:
    //  TaskXXX(std::shared_ptr<TaskEnvironment> env)
    //
    typedef TaskDefinitionPtr (*TaskFactory)(std::shared_ptr<TaskEnvironment>&);
    typedef const char * (*EnvCheckSum)();
#define DYNAMIC_TASK(T) extern "C" {\
    const char * getEnvironmentCheckSum() { return ENV_CHECK_SUM_STR(ENV_CHECK_SUM); } \
    task_manager_lib::TaskDefinitionPtr TaskFactoryObject(task_manager_lib::TaskEnvironmentPtr environment) {\
        return TaskDefinitionPtr(new T(environment));\
    } \
}


}

#endif // TASK_DEFINITION_H
