#ifndef TASK_ENVIRONMENT_H
#define TASK_ENVIRONMENT_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include <thread>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "task_manager_lib/ServiceStorage.h"

#ifndef ENV_CHECK_SUM
#warning ENV_CHECK_SUM is not defined, please add it in CMakeLists.txt
#define ENV_CHECK_SUM env_default
#endif
#define CHECK_SUM_STRINGIFY(x)  #x
#define ENV_CHECK_SUM_STR(x) CHECK_SUM_STRINGIFY(x)


namespace task_manager_lib {
    class TaskInstanceBase;

    /**
     * Empty class, to be inherited for a specific application. The existence of
     * the class provides an easy way to use the dynamic_cast to check the type of
     * the argument.\
     * */
    class TaskEnvironment  {
        protected:
            friend class TaskInstanceBase;
        protected:
            // This mutex will be locked in all the task instance function
            // (initialize, iterate, terminate) for periodic tasks. However, 
            // for non-periodic tasks, it is not possible to lock the
            // environment forever, so the lock is taken only for initialize and
            // terminate, but the user needs to organise his/her own locking as
            // appropriate, e.g for a reader or a writer
            // boost::shared_lock<std::shared_mutex> guard(env_gen->environment_mutex);
            // boost::unique_lock<std::shared_mutex> guard(env_gen->environment_mutex);
            std::mutex environment_mutex;
            rclcpp::Node::SharedPtr node;
            ServiceStorage service_storage;
        public:
            TaskEnvironment(rclcpp::Node::SharedPtr node) : node(node), service_storage(node) {}
            virtual ~TaskEnvironment() {}
            rclcpp::Node::SharedPtr getNode() {return node;}
            rclcpp::Node::ConstSharedPtr getNode() const {return node;}
            ServiceStorage & services() {
                return service_storage;
            }

            virtual const char * getEnvironmentCheckSum() const = 0;

#define DECLARE_ENV_CHECKSUM \
            virtual const char * getEnvironmentCheckSum() const { return ENV_CHECK_SUM_STR(ENV_CHECK_SUM); } 
    };

    typedef std::shared_ptr<TaskEnvironment> TaskEnvironmentPtr;
    typedef std::shared_ptr<TaskEnvironment const> TaskEnvironmentConstPtr;


}

#endif // TASK_ENVIRONMENT_H
