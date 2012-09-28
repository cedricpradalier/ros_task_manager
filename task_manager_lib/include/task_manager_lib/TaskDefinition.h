#ifndef TASK_DEFINITION_H
#define TASK_DEFINITION_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <string>
#include <task_manager_msgs/TaskStatus.h>
#include <task_manager_msgs/TaskDescription.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Config.h>

class DynamicTask;

// Enum defined in TaskStatus msg
typedef unsigned int TaskStatus;
/**
 * Basic function to build the string representation of one of the status above
 * */
extern
const char * taskStatusToString(TaskStatus ts);

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
		TaskStatus taskStatus;

		/**
		 * Default timeout of the tasks, for each new cycle
		 * */
		double defaultTimeout;
		/**
		 * Timeout value for this cycle
		 * */
		double timeout;

        /**
         * description of the parameters
         * */
        dynamic_reconfigure::ConfigDescription parameterDescription;

        /**
         * value of the parameters
         * */
        dynamic_reconfigure::Config config;

	public:
		TaskDefinition(const std::string & tname, const std::string & thelp, 
				bool isperiodic, double deftTimeout) :
			name(tname), help(thelp), periodic(isperiodic), 
            taskStatus(task_manager_msgs::TaskStatus::TASK_NEWBORN), 
			defaultTimeout(deftTimeout), timeout(deftTimeout) {}
		virtual ~TaskDefinition() {
			// printf("Delete task '%s'\n",name.c_str());
		}


		virtual void setName(const std::string & n);
		virtual const std::string & getName() const;
		virtual const std::string & getHelp() const;

		virtual bool isPeriodic() const;

		virtual double getTimeout() const;

		virtual void resetStatus();
		virtual TaskStatus getStatus() const;
		virtual const std::string & getStatusString() const;

		void doConfigure(const dynamic_reconfigure::Config & parameters);

		void doInitialise(const dynamic_reconfigure::Config & parameters);

		void doIterate();

		void doTerminate();

		void debug(const char *stemplate,...) const; 

        task_manager_msgs::TaskDescription getDescription() const;
        task_manager_msgs::TaskStatus getRosStatus() const;

        virtual dynamic_reconfigure::Config getParametersFromServer() = 0;

	protected:
		friend class DynamicTask;

		void setHelp(const std::string & h) {help = h;}
		void setPeriodic(bool p) {periodic = p;}
		void setTimeout(double tout) {timeout = tout;}
		void setStatusString(const std::string & s) {
			statusString = s;
		}


		virtual TaskStatus configure(const dynamic_reconfigure::Config & parameters) throw (InvalidParameter) = 0;

		virtual TaskStatus initialise(const dynamic_reconfigure::Config & parameters) throw (InvalidParameter) = 0;

		virtual TaskStatus iterate() = 0;

		virtual TaskStatus terminate() = 0;

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
