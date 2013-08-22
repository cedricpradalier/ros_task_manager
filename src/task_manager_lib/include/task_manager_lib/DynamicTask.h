#ifndef DYNAMIC_TASK_DEFINITION_H
#define DYNAMIC_TASK_DEFINITION_H

#include <string>


#include "TaskDefinition.h"

namespace task_manager_lib {
    // Specialisation of the TaskDefinition to use a task dynamically loaded from a
    // library. Note that such a dynamically loaded class must have a constructor
    // with the following profile:
    //    TaskXXX(boost::shared_ptr<TaskEnvironment> env)
    class DynamicTask : public TaskDefinitionBase
    {
        protected:
            // DLL file
            std::string filename;
            // Pointer to the dll object
            void * handle;
            // Shared pointer on the loaded task
            boost::shared_ptr<TaskDefinitionBase> task;

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

            virtual TaskInstancePtr instantiate() {
                return task->instantiate();
            }

            virtual void setName(const std::string & n) {task->setName(n);}
            virtual const std::string & getName() const {return task->getName();}
            virtual const std::string & getHelp() const {return task->getHelp();}
            virtual bool isPeriodic() const {return task->isPeriodic();}
            virtual void setTaskId(unsigned int id) {task->setTaskId(id);}
            virtual unsigned int getTaskId() {return task->getTaskId();}

            virtual TaskIndicator getStatus() const {
                //printf("DynamicTask: status of %s: %s\n",name.c_str(),taskStatusToString(task->getStatus()));
                return task->getStatus();
            }
            virtual void setStatus(const TaskIndicator & ti) {
                task->setStatus(ti);
            }

            virtual const std::string & getStatusString() const {
                return task->getStatusString();
            }
            virtual void setStatusString(const std::string & s) {
                task->setStatusString(s);
            }

            virtual TaskIndicator configure(const TaskParameters & parameters) throw (InvalidParameter) {
                task->doConfigure(taskId,parameters);
                return task->getStatus();
            }

            // Get the task description as a combination of task-specific
            // information and dynamic_reconfigure::ConfigDescription (assuming the
            // task as a config file)
            virtual task_manager_msgs::TaskDescription getDescription() const {
                return task->getDescription();
            }

            // Return the parameters as read from the parameter server. Returns the
            // default parameters otherwise.
            virtual TaskParameters getParametersFromServer(const ros::NodeHandle &nh) {
                return task->getParametersFromServer(nh);
            }

            // Return the default parameters, typically from the default value
            // defined in the .cfg file.
            virtual TaskParameters getDefaultParameters() const {
                return task->getDefaultParameters();
            }

            /**
             * description of the parameters
             * */
            virtual dynamic_reconfigure::ConfigDescription getParameterDescription() const {
                return task->getParameterDescription();
            }


    };


};


#endif // DYNAMIC_TASK_DEFINITION_H
