#ifndef DYNAMIC_TASK_DEFINITION_H
#define DYNAMIC_TASK_DEFINITION_H

#include <string>


#include "TaskDefinition.h"

class DynamicTask : public TaskDefinition
{
	protected:
		std::string filename;
		void * handle;
		TaskDefinition *task;

		struct DLLoadError : public std::exception {
			std::string text;
			DLLoadError(const std::string & s) {
				text = "DLLoadError: " + s;
			}
			virtual ~DLLoadError() throw () {}
			virtual const char *what() const throw () {
				return text.c_str();
			}
		};

	public:
		DynamicTask(const std::string & fname, TaskEnvironment *env);
		virtual ~DynamicTask();

		virtual void setName(const std::string & n) {task->setName(n);}
		virtual const std::string & getName() const {return task->getName();}
		virtual const std::string & getHelp() const {return task->getHelp();}
		virtual bool isPeriodic() const {return task->isPeriodic();}

		virtual double getTimeout() const {return task->getTimeout();}
		virtual void resetStatus() {
			task->resetStatus();
		}
		virtual TaskStatus getStatus() const {
			//printf("DynamicTask: status of %s: %s\n",name.c_str(),taskStatusToString(task->getStatus()));
			return task->getStatus();
		}
		virtual const std::string & getStatusString() const {
			return task->getStatusString();
		}

        virtual dynamic_reconfigure::Config getParametersFromServer() {
            return task->getParametersFromServer();
        }

		virtual TaskStatus configure(const dynamic_reconfigure::Config & parameters) throw (InvalidParameter) {
			task->doConfigure(parameters);
			return task->getStatus();
		}

		virtual TaskStatus initialise(const dynamic_reconfigure::Config & parameters) throw (InvalidParameter) {
			task->doInitialise(parameters);
			return task->getStatus();
		}

		virtual TaskStatus iterate() {
			task->doIterate();
			return task->getStatus();
		}

		virtual TaskStatus terminate() {
			task->doTerminate();
			return task->getStatus();
		}
};




#endif // DYNAMIC_TASK_DEFINITION_H
