#ifndef DYNAMIC_TASK_DEFINITION_H
#define DYNAMIC_TASK_DEFINITION_H

#include <string>


#include "TaskDefinition.h"

namespace task_manager_lib {
    // Specialisation of the TaskDefinition to use a task dynamically loaded from a
    // library. Note that such a dynamically loaded class must have a constructor
    // with the following profile:
    //    TaskXXX(boost::shared_ptr<TaskEnvironment> env)
    class DynamicTask : public TaskDefinition
    {
        protected:
            // DLL file
            std::string filename;
            // Pointer to the dll object
            void * handle;
            // Shared pointer on the loaded task
            boost::shared_ptr<TaskDefinition> task;

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
            // Load a class from a library file fname, and pass env to its
            // constructor. 
            DynamicTask(const std::string & fname, boost::shared_ptr<TaskEnvironment> env);
            virtual ~DynamicTask();

            //
            // All virtual function below are just forwarding their function to the
            // loaded class.
            //

            virtual boost::shared_ptr<TaskDefinition> getInstance() {
                return task->getInstance();
            }

            virtual void setName(const std::string & n) {task->setName(n);}
            virtual const std::string & getName() const {return task->getName();}
            virtual const std::string & getHelp() const {return task->getHelp();}
            virtual const TaskParameters & getConfig() const {return task->getConfig();}
            virtual bool isPeriodic() const {return task->isPeriodic();}
            virtual void setRuntimeId(unsigned int id) {task->setRuntimeId(id);}
            virtual void setTaskId(unsigned int id) {task->setTaskId(id);}
            virtual unsigned int getRuntimeId() {return task->getRuntimeId();}
            virtual unsigned int getTaskId() {return task->getTaskId();}

            virtual double getTimeout() const {return task->getTimeout();}
            virtual void resetStatus() {
                task->resetStatus();
            }
            virtual TaskIndicator getStatus() const {
                //printf("DynamicTask: status of %s: %s\n",name.c_str(),taskStatusToString(task->getStatus()));
                return task->getStatus();
            }
            virtual const std::string & getStatusString() const {
                return task->getStatusString();
            }

            virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const {
                return task->getParameterDescription();
            }

            virtual TaskParameters getDefaultParameters() const {
                return task->getDefaultParameters();
            }

            virtual TaskParameters getParametersFromServer(const ros::NodeHandle & nh) {
                return task->getParametersFromServer(nh);
            }

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter) {
                task->doConfigure(taskId,parameters);
                return task->getStatus();
            }

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter) {
                task->doInitialise(runId,parameters);
                return task->getStatus();
            }

            virtual TaskIndicator iterate() {
                task->doIterate();
                return task->getStatus();
            }

            virtual TaskIndicator terminate() {
                task->doTerminate();
                return task->getStatus();
            }
    };


};


#endif // DYNAMIC_TASK_DEFINITION_H
