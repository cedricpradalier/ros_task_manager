#ifndef TASK_ACTION_GENERIC_H
#define TASK_ACTION_GENERIC_H


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <task_manager_lib/TaskConfig.h>
#include <task_manager_lib/TaskInstance.h>


namespace task_manager_lib {
    struct TaskActionGenericWithoutClientConfig : public task_manager_lib::TaskConfig {
        TaskActionGenericWithoutClientConfig(double defautServerTimeout)  : task_manager_lib::TaskConfig() {
            define("server_timeout",defautServerTimeout,"How long to wait for the server to be ready",true);
        }

    };

    struct TaskActionGenericConfig : public TaskActionGenericWithoutClientConfig {
        TaskActionGenericConfig(const std::string & defaultActionName,double defautServerTimeout=5.0) : TaskActionGenericWithoutClientConfig(defautServerTimeout) {
            define("action_name",defaultActionName,"Name of the action server",true);
        }

    };

    
    // Class Config should inherit from TaskActionGenericWithoutClientConfig
    template <class Action, class Config,class Environment>
    class TaskActionGenericWithoutClient : public task_manager_lib::TaskInstance<Config,Environment>
    {
        protected:
            typedef task_manager_lib::TaskInstance<Config,Environment> Parent;


            using GoalHandleGeneric = rclcpp_action::ClientGoalHandle<Action>;
            typedef enum {
                TASK_WAITING_ACTION_SERVER,
                TASK_WAITING_GOAL_RESPONSE,
                TASK_WAITING_RESULT,
                TASK_GOAL_REJECTED,
                TASK_GOAL_FAILED,
                TASK_GOAL_SUCCEEDED
            } TaskState;

            rclcpp::Time t0;
            TaskState state;

            typedef typename rclcpp_action::Client<Action>::SharedPtr ClientPtr;
            ClientPtr this_action_client;
            typedef typename Action::Goal Goal;
            typedef typename Action::Feedback Feedback;
            typedef typename Action::Result Result;
            Goal goal_msg;

            // This must be declared by inheriting class
            virtual void buildActionGoal(Goal& goal) = 0;
            virtual ClientPtr getActionClient() = 0;

            // This can be overwritten if needed
            virtual void handleFeedback(const std::shared_ptr<const Feedback> /*feedback*/) {};
            virtual void handleResult(std::shared_ptr<Result> /*result*/) {};

            void sendGoal() {
                buildActionGoal(goal_msg);
                typename rclcpp_action::Client<Action>::SendGoalOptions send_goal_options;
                send_goal_options.goal_response_callback =
                    std::bind(&TaskActionGenericWithoutClient<Action,Config,Environment>::goal_response_callback, this, std::placeholders::_1);
                send_goal_options.feedback_callback =
                    std::bind(&TaskActionGenericWithoutClient<Action,Config,Environment>::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                send_goal_options.result_callback =
                    std::bind(&TaskActionGenericWithoutClient<Action,Config,Environment>::result_callback, this, std::placeholders::_1);
                this_action_client->async_send_goal(goal_msg, send_goal_options);
            }

              void goal_response_callback(const typename GoalHandleGeneric::SharedPtr & goal_handle) {
                  if (!goal_handle) {
                      RCLCPP_ERROR(this->getNode()->get_logger(), "Goal was rejected by server");
                      state = TASK_GOAL_REJECTED;
                  } else {
                      RCLCPP_INFO(this->getNode()->get_logger(), "Goal accepted by server, waiting for result");
                      state = TASK_WAITING_RESULT;
                  }
              }

              void feedback_callback(typename GoalHandleGeneric::SharedPtr,
                      const std::shared_ptr<const Feedback> feedback) {
                  this->handleFeedback(feedback);
              }

              void result_callback(const typename GoalHandleGeneric::WrappedResult & result) {
                  switch (result.code) {
                      case rclcpp_action::ResultCode::SUCCEEDED:
                          this->handleResult(result.result);
                          state = TASK_GOAL_SUCCEEDED;
                          break;
                      case rclcpp_action::ResultCode::ABORTED:
                          RCLCPP_ERROR(this->getNode()->get_logger(), "Goal was aborted");
                          state = TASK_GOAL_FAILED;
                          return;
                      case rclcpp_action::ResultCode::CANCELED:
                          RCLCPP_ERROR(this->getNode()->get_logger(), "Goal was canceled");
                          state = TASK_GOAL_FAILED;
                          return;
                      default:
                          RCLCPP_ERROR(this->getNode()->get_logger(), "Unknown result code");
                          state = TASK_GOAL_FAILED;
                          return;
                  }
              }

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskActionGenericWithoutClient(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                task_manager_lib::TaskInstance<Config,Environment>(def,ev){ }
            virtual ~TaskActionGenericWithoutClient() { 
            };

            /// Record the starting time
            virtual task_manager_lib::TaskIndicator initialise() {
                this_action_client = this->getActionClient();
                if (!this_action_client) {
                    RCLCPP_ERROR(this->getNode()->get_logger(),"TaskActionGenericWithoutClient::initialise called without creating client. Aborting");
                    return task_manager_lib::TaskStatus::TASK_INITIALISATION_FAILED;
                }
                state = TASK_WAITING_ACTION_SERVER;
                t0 = this->getNode()->get_clock()->now();
                return task_manager_lib::TaskStatus::TASK_INITIALISED;
            }

            virtual task_manager_lib::TaskIndicator iterate() {
                switch (state) {
                    case TASK_WAITING_ACTION_SERVER:
                        if (!this_action_client->action_server_is_ready()) {
                            if ((this->getNode()->get_clock()->now() - t0).seconds() > 
                                    task_manager_lib::TaskInstanceBase::getConfig()->get<double>("server_timeout")) {
                                RCLCPP_ERROR(this->getNode()->get_logger(),"TaskActionGeneric::initialise server is not becoming ready");
                                return task_manager_lib::TaskStatus::TASK_FAILED;
                            } else {
                                return task_manager_lib::TaskStatus::TASK_RUNNING;
                            }
                        }
                        state = TASK_WAITING_GOAL_RESPONSE;
                        sendGoal();
                        // fallthrough
                    case TASK_WAITING_GOAL_RESPONSE:
                        return task_manager_lib::TaskStatus::TASK_RUNNING;
                    case TASK_WAITING_RESULT:
                        return task_manager_lib::TaskStatus::TASK_RUNNING;
                    case TASK_GOAL_SUCCEEDED:
                        return task_manager_lib::TaskStatus::TASK_COMPLETED;
                    case TASK_GOAL_FAILED:
                    default:
                        return task_manager_lib::TaskStatus::TASK_FAILED;
                }
            }

            virtual task_manager_lib::TaskIndicator terminate() {
                if (this_action_client) {
                    switch (state) {
                        case TASK_WAITING_GOAL_RESPONSE:
                        case TASK_WAITING_RESULT:
                            this_action_client->async_cancel_all_goals();
                            break;
                        case TASK_GOAL_SUCCEEDED:
                        case TASK_GOAL_FAILED:
                        case TASK_WAITING_ACTION_SERVER:
                        default:
                            break;
                    }
                }
                return task_manager_lib::TaskStatus::TASK_TERMINATED;
            }

    };

    // Class Config should inherit from TaskActionGenericConfig or define action_name if 
    // getActionName is not redefined
    template <class Action, class Config,class Environment>
    class TaskActionGeneric : public TaskActionGenericWithoutClient<Action,Config,Environment>
    {
        protected:
            typedef TaskActionGenericWithoutClient<Action,Config,Environment> Parent;

            // This must be declared by inheriting class
            virtual std::string getActionName() const {
                return task_manager_lib::TaskInstanceBase::getConfig()->get<std::string>("action_name");
            }

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskActionGeneric(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                TaskActionGenericWithoutClient<Action,Config,Environment>(def,ev) { }
            virtual ~TaskActionGeneric() { };

            virtual typename Parent::ClientPtr getActionClient() {
                return rclcpp_action::create_client<Action>( this->getNode(), this->getActionName());
            }
    };

}





#endif // TASK_ACTION_GENERIC_H
