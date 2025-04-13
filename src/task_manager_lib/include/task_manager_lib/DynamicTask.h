#ifndef DYNAMIC_TASK_DEFINITION_H
#define DYNAMIC_TASK_DEFINITION_H

#include <string>


#include "task_manager_lib/TaskDefinition.h"

namespace task_manager_lib {
    // Specialisation of the TaskDefinition to use a task dynamically loaded from a
    // library. Note that such a dynamically loaded class must have a constructor
    // with the following profile:
    //    TaskXXX(boost::shared_ptr<TaskEnvironment> env)
    class DynamicTask : public TaskDefinitionBase
    {
        public: 
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

        protected:
            // DLL file
            std::string filename;
            // Pointer to the dll object
            void * handle;
            // Shared pointer on the loaded task
            TaskDefinitionPtr task;

        public:
            // Load a class from a library file fname, and pass env to its
            // constructor. 
            DynamicTask(const std::string & fname, TaskEnvironmentPtr env);
            virtual ~DynamicTask();

            bool loadTask(bool exceptions);

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
            virtual void setHelp(const std::string & h) {task->setHelp(h);}
            virtual bool isPeriodic() const {return task->isPeriodic();}
            virtual void setTaskId(int id) {task->setTaskId(id);}
            virtual int getTaskId() {return task->getTaskId();}

            virtual task_manager_msgs::msg::TaskDescription getDescription() const {
                return task->getDescription();
            }
            virtual TaskEnvironmentPtr getEnvironment() {return task->getEnvironment();}
            virtual TaskConfigPtr getConfig() {return task->getConfig();}
            virtual TaskConfigConstPtr getConfig() const {return task->getConfig();}
            virtual std::string getInstanceName() {
                return task->getInstanceName();
            }

    };


}


#endif // DYNAMIC_TASK_DEFINITION_H
