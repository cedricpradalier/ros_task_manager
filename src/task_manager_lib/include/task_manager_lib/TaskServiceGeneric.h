#ifndef TASK_SERVER_GENERIC_H
#define TASK_SERVER_GENERIC_H

#include "rclcpp/rclcpp.hpp"
#include "task_manager_lib/TaskConfig.h"
#include "task_manager_lib/TaskInstance.h"

namespace task_manager_lib {
    struct TaskServiceGenericWithoutClientConfig : public task_manager_lib::TaskConfig {
        TaskServiceGenericWithoutClientConfig(double defautServerTimeout=5.0)  : task_manager_lib::TaskConfig() {
            define("server_timeout",defautServerTimeout,"How long to wait for the server to be ready",true);
        }
    };

    struct TaskServiceGenericConfig : public TaskServiceGenericWithoutClientConfig {
        TaskServiceGenericConfig(const std::string & defaultServiceName,double defautServerTimeout=5.0) : TaskServiceGenericWithoutClientConfig(defautServerTimeout) {
            define("service_name",defaultServiceName,"Name of the service",true);
        }
    };

    template <class Service, class Config,class Environment>
    class TaskServiceGenericWithoutClient : public TaskInstance<Config,Environment>
    {
        protected:
            typedef TaskServiceGenericWithoutClient<Service,Config,Environment> Parent;

            typedef enum {
                WAITING_FOR_CLIENT_SERVER,
                WAITING_FOR_FUTURE
            } ClientState;
            typedef typename Service::Request Request;
            typedef typename Service::Response Response;
            typedef typename rclcpp::Client<Service>::SharedPtr ClientPtr;

            rclcpp::Time t0;
            ClientState state;
            typename rclcpp::Client<Service>::SharedFuture future;
            ClientPtr this_service_client;
            
            // This must be declared by inheriting class
            virtual void buildServiceRequest(Request& /*goal*/) {}
            virtual TaskIndicator handleResponse(const Response& /*res*/) {
                return TaskStatus::TASK_COMPLETED;
            }

            virtual ClientPtr getServiceClient() = 0;

            void sendRequest() {
                auto request = std::make_shared<typename Service::Request>();
                this->buildServiceRequest(*request);
                future = this_service_client->async_send_request(request).future.share();
            }

        public:
            TaskServiceGenericWithoutClient(TaskDefinitionPtr def, TaskEnvironmentPtr env) : TaskInstance<Config,Environment>(def,env) {}
            virtual ~TaskServiceGenericWithoutClient() {};

            virtual TaskIndicator initialise() {
                this_service_client = this->getServiceClient();
                if (!this_service_client) {
                    RCLCPP_ERROR(this->getNode()->get_logger(),"TaskServiceGenericWithoutClient::initialise called without creating client. Aborting");
                    return task_manager_lib::TaskStatus::TASK_INITIALISATION_FAILED;
                }
                state = WAITING_FOR_CLIENT_SERVER;
                t0 = this->getNode()->get_clock()->now();
                return task_manager_lib::TaskStatus::TASK_INITIALISED;
            }

            virtual TaskIndicator iterate() {
                switch (state) {
                    case WAITING_FOR_CLIENT_SERVER:
                        if (!this_service_client->service_is_ready()) {
                            if ((this->getNode()->get_clock()->now() - t0).seconds() > 
                                    task_manager_lib::TaskInstanceBase::getConfig()->get<double>("server_timeout")) {
                                RCLCPP_ERROR(this->getNode()->get_logger(),"TaskServiceGenericWithoutClient::initialise server is not becoming ready");
                                return task_manager_lib::TaskStatus::TASK_FAILED;
                            } else {
                                return task_manager_lib::TaskStatus::TASK_RUNNING;
                            }
                        }
                        state = WAITING_FOR_FUTURE;
                        sendRequest(); // create future
                        // fallthrough
                    case WAITING_FOR_FUTURE:
                        if (!future.valid()) {
                            RCLCPP_ERROR(this->getNode()->get_logger(),"TaskServiceGenericWithoutClient: No Future");
                            return TaskStatus::TASK_FAILED;
                        }
                        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                            RCLCPP_DEBUG(this->getNode()->get_logger(),"TaskServiceGenericWithoutClient: received future");
                            return this->handleResponse(*(future.get()));
                        }
                        break;
                }
                return TaskStatus::TASK_RUNNING;
            }
    };

    // Class Config should inherit from TaskServiceGenericConfig or define service_name if 
    // getServiceName is not redefined
    template <class Service, class Config,class Environment>
    class TaskServiceGeneric : public TaskServiceGenericWithoutClient<Service,Config,Environment>
    {
        protected:
            typedef TaskServiceGeneric<Service,Config,Environment> Parent;

            // This must be declared by inheriting class
            virtual std::string getServiceName() const {
                return task_manager_lib::TaskInstanceBase::getConfig()->get<std::string>("service_name");
            }

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskServiceGeneric(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                TaskServiceGenericWithoutClient<Service,Config,Environment>(def,ev) { }
            virtual ~TaskServiceGeneric() { };

            virtual typename rclcpp::Client<Service>::SharedPtr getServiceClient() {
                std::string name = this->getServiceName();
                RCLCPP_INFO(this->getNode()->get_logger(),"Task %s: creating client %s",
                        this->getName().c_str(),name.c_str());
                return this->getEnvironment()->services().template registerServiceClient<Service>(name);
            }
    };
}

#endif // TASK_SERVER_GENERIC_H
