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

namespace task_manager_lib {
    /**
     * Empty class, to be inherited for a specific application. The existence of
     * the class provides an easy way to use the dynamic_cast to check the type of
     * the argument.\
     * */
    class TaskEnvironment  {
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
            rclcpp::Node::SharedPtr node;
        public:
            TaskEnvironment(rclcpp::Node::SharedPtr node) : node(node) {}
            virtual ~TaskEnvironment() {}
            rclcpp::Node::SharedPtr getNode() {
                return node;
            }
    };

    typedef std::shared_ptr<TaskEnvironment> TaskEnvironmentPtr;
    typedef std::shared_ptr<TaskEnvironment const> TaskEnvironmentConstPtr;


}

#endif // TASK_ENVIRONMENT_H
