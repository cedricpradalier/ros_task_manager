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

// #define USE_ENCAPSULATED
#ifdef USE_ENCAPSULATED
#include <task_manager_msgs/EncapsulatedMessage.h>
#include <task_manager_msgs/encapsulate_message.h>
#else
namespace task_manager_msgs {
    // Placeholder
    namespace msg {
        struct EncapsulatedMessage {
        };
    }
}
#endif

// #include <dynamic_reconfigure/ConfigDescription.h>
// #include <dynamic_reconfigure/server.h>
// #include <dynamic_reconfigure/Config.h>
// #include <dynamic_reconfigure/config_tools.h>

namespace task_manager_lib {
    class DynamicTask;

    // Enum defined in TaskStatus msg
    typedef unsigned int TaskIndicator;

    /**
     * Empty class, to be inherited for a specific application. The existence of
     * the class provides an easy way to use the dynamic_cast to check the type of
     * the argument.\
     * */
    class TaskEnvironment : public rclcpp::Node {
        public:
            // This mutex will be locked in all the task instance function
            // (initialize, iterate, terminate) for periodic tasks. However, 
            // for non-periodic tasks, it is not possible to lock the
            // environment forever, so the lock is taken only for initialize and
            // terminate, but the user needs to organise his/her own locking as
            // appropriate, e.g for a reader or a writer
            // boost::shared_lock<std::shared_mutex> guard(env_gen->environment_mutex);
            // boost::unique_lock<std::shared_mutex> guard(env_gen->environment_mutex);
            std::mutex environment_mutex;
        public:
            TaskEnvironment(const std::string & envNodeName) : rclcpp::Node(envNodeName) {}
            virtual ~TaskEnvironment() {}
    };

    typedef std::shared_ptr<TaskEnvironment> TaskEnvironmentPtr;
    typedef std::shared_ptr<TaskEnvironment const> TaskEnvironmentConstPtr;

    class TaskInstanceBase;
    typedef std::shared_ptr<TaskInstanceBase> TaskInstancePtr;
    typedef std::shared_ptr<TaskInstanceBase const> TaskInstanceConstPtr;
    
    

    class TaskParameterDefinitionBase {
        protected:
            rclcpp::ParameterValue value;
            rclcpp::ParameterValue defaultValue;
            rcl_interfaces::msg::ParameterDescriptor descriptor;
        public:
            template <class ParameterT> 
                TaskParameterDefinitionBase(const std::string & name,
                        const ParameterT & val,
                        const std::string & description,
                        bool read_only) : value(val), defaultValue(val) {
                    descriptor.name = name;
                    descriptor.type = value.get_type();
                    descriptor.description = description;
                    descriptor.read_only = read_only;
                }
            
            template <class ParameterT> 
                ParameterT get() const {
                    return value.get<ParameterT>();
                }

            void setDefaultValue() {
                value = defaultValue;
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

    class TaskConfig {
        protected:
            typedef std::map<std::string,TaskParameterDefinitionBase*> TaskConfigMap;
            TaskConfigMap definitions;

        public:
            TaskParameterDefinitionBase task_rename;
            TaskParameterDefinitionBase foreground;
            TaskParameterDefinitionBase task_period;
            TaskParameterDefinitionBase task_timeout;

        public:
            TaskConfig() : 
                task_rename("task_rename","","used to rename a task at runtime [deprecated]",true),
                foreground("foreground",true,"run this task in foreground or not",true),
                task_period("task_period",1.0,"default task period for periodic task",true),
                task_timeout("task_timeout",-1.0,"if positive, maximum time this task can run for",true) {
                    definitions["task_rename"]=&task_rename;
                    definitions["foreground"]=&foreground;
                    definitions["task_period"]=&task_period;
                    definitions["task_timeout"]=&task_timeout;
                }
            virtual ~TaskConfig() {}

            bool registerParameter(const std::string & name, TaskParameterDefinitionBase * definition) {
                definitions[name] = definition;
                return true;
            }

            bool declareParameters(rclcpp::Node * node);
            void updateParameters(rclcpp::Node * node);
            void publishParameters(rclcpp::Node * node);

            std::vector<rcl_interfaces::msg::ParameterDescriptor> getParameterDescriptions();

            void loadConfig(const task_manager_msgs::msg::TaskConfig & cfg) {
                for (size_t i=0;i<cfg.plist.size();i++) {
                    std::string name = cfg.plist[i].name;
                    TaskConfigMap::iterator it = definitions.find(name);
                    if (it == definitions.end()) {
                        continue;
                    }
                    it->second->setValue(cfg.plist[i].value);
                }
            }

            void loadConfig(const TaskConfig & cfg) {
                for (TaskConfigMap::const_iterator it=cfg.definitions.begin();it!=cfg.definitions.end();it++) {
                    definitions.insert(*it);
                }
            }

            void loadConfig(const std::vector<rclcpp::Parameter> & cfg) {
                for (std::vector<rclcpp::Parameter>::const_iterator it=cfg.begin();it!=cfg.end();it++) {
                    std::string name = it->get_name();
                    TaskConfigMap::iterator dit = definitions.find(name);
                    if (dit == definitions.end()) {
                        continue;
                    }
                    dit->second->setValue(it->get_parameter_value());
                }
            }

            bool getDefinition(const std::string & name,TaskParameterDefinitionBase & def) {
                TaskConfigMap::iterator it = definitions.find(name);
                if (it == definitions.end()) {
                    return false;
                }
                def = *(it->second);
                return true;
            }
    };

    class TaskParameterDefinition : public TaskParameterDefinitionBase {
        public:
            template <class ParameterT> 
                TaskParameterDefinition(TaskConfig*config,
                        const std::string & name,
                        const ParameterT & defaultValue,
                        const std::string & description,
                        bool read_only) : TaskParameterDefinitionBase(name,defaultValue,description,read_only) {
                    config->registerParameter(name,this);
                }
    };

    typedef std::shared_ptr<TaskConfig> TaskConfigPtr;
    typedef std::shared_ptr<TaskConfig const> TaskConfigConstPtr;


#if 0
    // Extension of the task_manager_msgs::TaskConfig class to add more 
    // object like functions and conversion
    typedef std::map<std::string,rclcpp::ParameterValue> ParameterMap;
    class TaskParameters: public ParameterMap {
        protected:
            std::string prefix;
        public:
            TaskParameters() { }
            TaskParameters(const task_manager_msgs::msg::TaskConfig & cfg) {
                loadConfig(cfg);
            }
            TaskParameters(const TaskParameters & cfg) 
                : ParameterMap(cfg) {}

            void loadConfig(const task_manager_msgs::msg::TaskConfig & cfg,bool append_prefix=false) {
                for (size_t i=0;i<cfg.plist.size();i++) {
                    std::string name = cfg.plist[i].name;
                    if (append_prefix) {
                        name = prefix + name;
                    }
                    insert(ParameterMap::value_type(name, cfg.plist[i].value));
                }
            }

            void loadConfig(const TaskParameters & cfg,bool append_prefix=false) {
                for (ParameterMap::const_iterator it=cfg.begin();it!=cfg.end();it++) {
                    if (append_prefix) {
                        std::string name = it->first;
                        name = prefix + name;
                        insert(ParameterMap::value_type(name,it->second));
                    } else {
                        insert(*it);
                    }
                }
            }

            void loadConfig(const std::vector<rclcpp::Parameter> & cfg,bool append_prefix=false) {
                for (std::vector<rclcpp::Parameter>::const_iterator it=cfg.begin();it!=cfg.end();it++) {
                    if (append_prefix) {
                        set(prefix+it->get_name(),it->get_parameter_value());
                    } else {
                        set(it->get_name(),it->get_parameter_value());
                    }
                }
            }

            void setPrefix(const std::string & pfix) {
                prefix = pfix;
            }

            const std::string & getPrefix() const {
                return prefix;
            }

            iterator find(const std::string & name, bool append_prefix) {
                return ParameterMap::find(append_prefix?(prefix+name):name);
            }

            const_iterator find(const std::string & name, bool append_prefix) const {
                return ParameterMap::find(append_prefix?(prefix+name):name);
            }

            // Read a parameter from the structure, and return false if it is
            // missing. Mostly a convenience function to avoid the long namespaces.
            template <class T>
                bool get(const std::string &name, T &val) const
                {
                    ParameterMap::const_iterator it = ParameterMap::find(prefix+name); 
                    if (it==end()) {
                        return false;
                    }
                    val = it->second.get<T>();
                    return true;
                }

            // Set a parameter ino the structure. This is missing in the
            // ConfigTools class.
            template <class T>
                void set(const std::string &name, const T &val)
                {
                    insert(ParameterMap::value_type(prefix+name,rclcpp::ParameterValue(val)));
                }

            void set(const rclcpp::Parameter &val, bool append_prefix=false)
            {
                if (append_prefix) {
                    insert(ParameterMap::value_type(prefix+val.get_name(),val.get_parameter_value()));
                } else {
                    insert(ParameterMap::value_type(val.get_name(),val.get_parameter_value()));
                }
            }

            // Merge a set of parameters with the current one. Any parameter in
            // tnew that is already in 'this' is updated. If it is not there yet,
            // it is inserted.
            void update(const TaskParameters & tnew) {
                loadConfig(tnew);
            }

#if 0
            // Convert the parameter to a XXXConfig class, as generated by an .cfg
            // file. See dynamic_reconfigure/templates/TypeConfig.h
            template <class CFG>
                CFG toConfig() const {
                    CFG cfg = CFG::__getDefault__();
                    task_manager_msgs::TaskConfig drc = *this;
                    cfg.__fromMessage__(drc);
                    return cfg;
                }

            // Convert the data from a XXXConfig class, to a parameter structure
            // suitable for sending as a ROS message. A bit too many copies in this
            // functions.
            template <class CFG>
                void fromConfig(const CFG & cfg) {
                    task_manager_msgs::TaskConfig drc = *this;
                    cfg.__toMessage__(drc);
                    *this = drc;
                }

            // Dump the content of the parameters. Mostly for debug
            void print(FILE * fp=stdout) const {
                for (unsigned int i = 0; i < bools.size(); i++) {
                    fprintf(fp,"bool: %s = %s\n",bools[i].name.c_str(),bools[i].value?"true":"false");
                }
                for (unsigned int i = 0; i < ints.size(); i++) {
                    fprintf(fp,"int : %s = %d\n",ints[i].name.c_str(),ints[i].value);
                }
                for (unsigned int i = 0; i < doubles.size(); i++) {
                    fprintf(fp,"dbl : %s = %e\n",doubles[i].name.c_str(),doubles[i].value);
                }
                for (unsigned int i = 0; i < strs.size(); i++) {
                    fprintf(fp,"str : %s = '%s'\n",strs[i].name.c_str(),strs[i].value.c_str());
                }
            }

#endif
    };
#endif

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

    // Forward declaration for getInstance
    class TaskInstanceBase;


    /**
     *
     * Mother class of all tasks. Contains all the generic tools to define a task.
     * Must be inherited. Such a task can be periodic or aperiodic
     * 
     * */
    class TaskDefinitionBase : public rclcpp::Node
    {
        protected:
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
                rclcpp::Node(tname), name(tname), help(thelp), periodic(isperiodic), 
                env_gen(ev), cfg_gen(cfg), taskId(-1) {
                    cfg_gen->declareParameters(this);
                }
            virtual ~TaskDefinitionBase() {
                // printf("Delete task '%s'\n",name.c_str());
                // fflush(stdout);
            }

            std::string getInstanceName() {
                char counter[128];
                sprintf(counter,"%8d",instanceCounter++);
                return name + counter;
            }


            // Set the task runtime id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual unsigned int getTaskId() const;

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

            // Provide an instance of the class (or a derivative of it), with
            // its own internal variables that can be run multiple time. 
            virtual TaskInstancePtr instantiate() = 0; 

            TaskEnvironmentPtr getEnvironment() {return env_gen;}
            TaskConfigPtr getConfig() {return cfg_gen;}
        public:
            // All the functions below are intended for the TaskScheduler.
            // Set the task id . Has to be virtual because it is overloaded by
            // the dynamic class proxy.
            virtual void setTaskId(unsigned int id);

            // Output a debut string, prefixed by the task name
            void debug(const char *stemplate,...) const; 

        protected:
            // Set of functions only useful for derived classes
            friend class DynamicTask;

            // Update the description string
            void setHelp(const std::string & h) {help = h;}
        protected:
            // Set of functions that must be implemented by any inheriting class

    };

    typedef std::shared_ptr<TaskDefinitionBase> TaskDefinitionPtr;
    typedef std::shared_ptr<TaskDefinitionBase const> TaskDefinitionConstPtr;

    /**
     *
     * Mother class of all tasks. Contains all the generic tools to define a task.
     * Must be inherited. Such a task can be periodic or aperiodic
     * 
     * */
    class TaskInstanceBase : public rclcpp::Node
    {
        protected:
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

            OnSetParametersCallbackHandle::SharedPtr setParamHandle;
            rcl_interfaces::msg::SetParametersResult
                reconfigure_callback(const std::vector<rclcpp::Parameter> & parameters)
                {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    // for (const auto & parameter : parameters) {
                    //     if (!some_condition) {
                    //         result.successful = false;
                    //         result.reason = "the reason it could not be allowed";
                    //     }
                    // }
                    cfg_gen->loadConfig(parameters); 
                    this->reconfigure();
                    return result;
                }

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
            TaskInstanceBase(TaskDefinitionPtr def, TaskEnvironmentPtr ev) :
                rclcpp::Node(def->getInstanceName()), definition(def), taskStatus(task_manager_msgs::msg::TaskStatus::TASK_CONFIGURED), 
                timeout(-1.0), runId(-1), env_gen(ev), cfg_gen(def->getConfig()) {
                    cfg_gen->declareParameters(this);
                    setParamHandle = this->add_on_set_parameters_callback(std::bind(&TaskInstanceBase::reconfigure_callback, this, std::placeholders::_1));

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
            bool isAnInstanceOf(const TaskDefinitionBase & def);
            bool isAnInstanceOf(TaskDefinitionConstPtr def);

            // Call the virtual initialise function, but prepare the class before
            // hand.
            void doInitialise(unsigned int runtimeId, const task_manager_msgs::msg::TaskConfig & parameters);

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

        protected:
            // Set of functions only useful for derived classes
            friend class DynamicTask;

        protected:
            // Set of functions that must be implemented by any inheriting class

            // Initialise is called once every time the task is launched
            virtual TaskIndicator initialise() {
                RCLCPP_INFO(this->get_logger(),"Initialising task %s: default function",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_INITIALISED;
            }

            // iterate is called only once for non periodic tasks. It is called
            // iteratively with period 'task_period' for periodic class. 
            virtual TaskIndicator iterate() {
                RCLCPP_INFO(this->get_logger(),"Task %s: default iteration",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_COMPLETED;
            }

            // Terminate is called once when the task is completed, cancelled or
            // interrupted.
            virtual TaskIndicator terminate() {
                RCLCPP_INFO(this->get_logger(),"Terminating task %s: default function",this->getName().c_str());
                return task_manager_msgs::msg::TaskStatus::TASK_TERMINATED;
            }

    };


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
                virtual ~TaskDefinition() {}


                virtual TaskInstancePtr instantiate() {
                    TaskDefinitionPtr def = std::dynamic_pointer_cast<TaskDefinition,rclcpp::Node>(shared_from_this());
                    return TaskInstancePtr(new INSTANCE(def,env_gen));
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
                    : TaskInstanceBase(def,ev) {
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


    // Function type for the TaskFactoryObject function that will be inserted into
    // each class to be used as a dynamic class (i.e. a .so library).
    //
    // Note that a class inherited from TaskDefinition and meant to be used with
    // the dynamic loading mecanism must have a constructor with the following
    // profile:
    //  TaskXXX(std::shared_ptr<TaskEnvironment> env)
    //
    typedef TaskDefinitionPtr (*TaskFactory)(std::shared_ptr<TaskEnvironment>&);
#define DYNAMIC_TASK(T) extern "C" {\
    task_manager_lib::TaskDefinitionPtr TaskFactoryObject(task_manager_lib::TaskEnvironmentPtr environment) {\
        return TaskDefinitionPtr(new T(environment));\
    } \
}


}

#endif // TASK_DEFINITION_H
