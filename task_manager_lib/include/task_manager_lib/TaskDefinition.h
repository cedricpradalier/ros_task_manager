#ifndef TASK_DEFINITION_H
#define TASK_DEFINITION_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ros/ros.h>

#include <string>
#include <task_manager_msgs/TaskStatus.h>
#include <task_manager_msgs/TaskDescription.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/config_tools.h>

class DynamicTask;

// Enum defined in TaskStatus msg
typedef unsigned int TaskIndicator;

class TaskParameters: public dynamic_reconfigure::Config {
    protected:
        template <class VT, class T>
            bool setParameter(std::vector<VT> &vec, const std::string &name, const T &val)
            {
                for (typename std::vector<VT>::iterator i = vec.begin(); i != vec.end(); i++)
                    if (i->name == name)
                    {
                        i->value = val;
                        return true;
                    }
                return false;
            }


    public:
        TaskParameters() 
            : dynamic_reconfigure::Config() { setDefaultParameters(); }
        TaskParameters(const dynamic_reconfigure::Config & cfg) 
            : dynamic_reconfigure::Config(cfg) {}
        TaskParameters(const TaskParameters & cfg) 
            : dynamic_reconfigure::Config(cfg) {}

        void setDefaultParameters() {
            setParameter("task_rename",std::string());
            setParameter("main_task",true);
            setParameter("task_period",-1.);
            setParameter("task_timeout",-1.);
        }


        template <class T>
            bool getParameter(const std::string &name, T &val) const
            {
                return dynamic_reconfigure::ConfigTools::getParameter(
                        dynamic_reconfigure::ConfigTools::getVectorForType(*this, val), name, val);
            }

        template <class T>
            void setParameter(const std::string &name, const T &val)
            {
                if (!setParameter(dynamic_reconfigure::ConfigTools::getVectorForType(*this, val), name, val)) {
                    dynamic_reconfigure::ConfigTools::appendParameter(*this,name,val);
                }
            }

        void update(const TaskParameters & tnew) {
            for (unsigned int i = 0; i < tnew.bools.size(); i++) {
                setParameter(tnew.bools[i].name,(bool)tnew.bools[i].value);
            }
            for (unsigned int i = 0; i < tnew.ints.size(); i++) {
                setParameter(tnew.ints[i].name,(int)tnew.ints[i].value);
            }
            for (unsigned int i = 0; i < tnew.doubles.size(); i++) {
                setParameter(tnew.doubles[i].name,(double)tnew.doubles[i].value);
            }
            for (unsigned int i = 0; i < tnew.strs.size(); i++) {
                setParameter(tnew.strs[i].name,tnew.strs[i].value);
            }
        }

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
};

/**
 * Basic function to build the string representation of one of the status above
 * */
extern
const char * taskStatusToString(TaskIndicator ts);

/**
 *
 * Mother class of all tasks. Contains all the generic tools to define a task.
 * Must be inherited. Such a task can be periodic or aperiodic
 * 
 * */
class TaskDefinition
{
	public:
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
         * value of the parameters
         * */
        TaskParameters config;

	public:
		TaskDefinition(const std::string & tname, const std::string & thelp, 
				bool isperiodic, double deftTimeout) :
			name(tname), help(thelp), periodic(isperiodic), 
            taskStatus(task_manager_msgs::TaskStatus::TASK_NEWBORN), 
			timeout(deftTimeout) {}
		virtual ~TaskDefinition() {
			printf("Delete task '%s'\n",name.c_str());
            fflush(stdout);
		}


		virtual void setName(const std::string & n);
		virtual const std::string & getName() const;
		virtual const std::string & getHelp() const;

		virtual bool isPeriodic() const;

		virtual double getTimeout() const;

		virtual void resetStatus();
		virtual TaskIndicator getStatus() const;
		virtual const std::string & getStatusString() const;
        virtual const TaskParameters & getConfig() const;

		void doConfigure(const TaskParameters & parameters);

		void doInitialise(const TaskParameters & parameters);

		void doIterate();

		void doTerminate();

		void debug(const char *stemplate,...) const; 

        task_manager_msgs::TaskDescription getDescription() const;
        task_manager_msgs::TaskStatus getRosStatus() const;

        virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh) = 0;
        virtual TaskParameters getDefaultParameters() const = 0;

        /**
         * description of the parameters
         * */
        virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const = 0;

	protected:
		friend class DynamicTask;

		void setHelp(const std::string & h) {help = h;}
		void setPeriodic(bool p) {periodic = p;}
		void setTimeout(double tout) {timeout = tout;}
		void setStatusString(const std::string & s) {
			statusString = s;
		}

		virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter) = 0;

		virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter) = 0;

		virtual TaskIndicator iterate() = 0;

		virtual TaskIndicator terminate() = 0;

};

template <class CFG>
class TaskDefinitionWithConfig : public TaskDefinition {
    public:
		TaskDefinitionWithConfig(const std::string & tname, const std::string & thelp, 
				bool isperiodic, double deftTimeout) :
            TaskDefinition(tname,thelp,isperiodic,deftTimeout) {}
        virtual ~TaskDefinitionWithConfig() {}

        virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const {
            return CFG::__getDescriptionMessage__();
        }

        virtual TaskParameters getDefaultParameters() const {
            TaskParameters tp;
            CFG cfg = CFG::__getDefault__();
            cfg.__toMessage__(tp);
            return tp;
        }


        virtual TaskParameters getParametersFromServer(const ros::NodeHandle & nh) {
            TaskParameters tp;
            CFG cfg = CFG::__getDefault__();
            cfg.__fromServer__(nh);
            cfg.__toMessage__(tp);
            return tp;
        }
};

/**
 * Empty class, to be inherited for a specific application. The existence of
 * the class provides an easy way to use the dynamic_cast to check the type of
 * the argument.\
 * */
class TaskEnvironment {
	public:
		TaskEnvironment() {}
		virtual ~TaskEnvironment() {}
};

typedef TaskDefinition* (*TaskFactory)(TaskEnvironment *);
#define DYNAMIC_TASK(T) extern "C" {\
	TaskDefinition* TaskFactoryObject(TaskEnvironment *environment) {\
		return new T(environment);\
	} \
}



#endif // TASK_DEFINITION_H
