#ifndef TASK_ACTION_GENERIC_H
#define TASK_ACTION_GENERIC_H


#include <task_manager_msgs/TaskStatus.h>
#include <task_manager_lib/TaskDefinition.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/action_definition.h>


namespace task_manager_action {
    
    template <class Action, class TaskConfig,class TaskEnvironment>
    class TaskActionGenericWithoutClient : public task_manager_lib::TaskInstance<TaskConfig,TaskEnvironment>
    {
        protected:
            ACTION_DEFINITION(Action)

            typedef actionlib::SimpleActionClient<Action> Client;
            typedef boost::shared_ptr<Client> ClientPtr;
            ClientPtr this_action_client;

            // This must be declared by inheriting class
            virtual void buildActionGoal(Goal& goal) const = 0;
            virtual ClientPtr getActionClient() = 0;

            // This can be overwritten if needed
            virtual void handleResult(ResultConstPtr result) {};

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskActionGenericWithoutClient(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                task_manager_lib::TaskInstance<TaskConfig,TaskEnvironment>(def,ev){ }
            virtual ~TaskActionGenericWithoutClient() { 
            };

            /// Record the starting time
            virtual task_manager_lib::TaskIndicator initialise() {
                Goal goal;
                this_action_client = this->getActionClient();
                if (!this_action_client) {
                    ROS_ERROR("TaskActionGenericWithoutClient::initialise called without creating client. Aborting");
                    
                    return task_manager_msgs::TaskStatus::TASK_INITIALISATION_FAILED;
                }
                buildActionGoal(goal);
                this_action_client->sendGoal(goal);
                return task_manager_msgs::TaskStatus::TASK_INITIALISED;
            }

            virtual task_manager_lib::TaskIndicator iterate() {
                switch (this_action_client->getState().state_) {
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        this->handleResult(this_action_client->getResult());
                        return task_manager_msgs::TaskStatus::TASK_COMPLETED;
                    case actionlib::SimpleClientGoalState::PENDING:
                    case actionlib::SimpleClientGoalState::ACTIVE:
                        return task_manager_msgs::TaskStatus::TASK_RUNNING;
                    default:
                        return task_manager_msgs::TaskStatus::TASK_FAILED;
                }
            }

            virtual task_manager_lib::TaskIndicator terminate() {
                if (this_action_client) {
                    switch (this_action_client->getState().state_) {
                        case actionlib::SimpleClientGoalState::PENDING:
                        case actionlib::SimpleClientGoalState::ACTIVE:
                            this_action_client->cancelGoal();
                            break;
                        default:
                            break;
                    }
                }
                return task_manager_msgs::TaskStatus::TASK_TERMINATED;
            }

    };

    template <class Action, class TaskConfig,class TaskEnvironment>
    class TaskActionGeneric : public TaskActionGenericWithoutClient<Action,TaskConfig,TaskEnvironment>
    {
        protected:
            typedef TaskActionGenericWithoutClient<Action,TaskConfig,TaskEnvironment> Parent;

            // This must be declared by inheriting class
            virtual const std::string & getActionName() const = 0;

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskActionGeneric(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                TaskActionGenericWithoutClient<Action,TaskConfig,TaskEnvironment>(def,ev) { }
            virtual ~TaskActionGeneric() { };

            virtual typename Parent::ClientPtr getActionClient() {
                typename Parent::ClientPtr client(new typename Parent::Client(this->getActionName(),true));
                if (!client->waitForServer(ros::Duration(5.0))) {
                    ROS_ERROR("Action server %s did not reply after 5s, aborting task",this->getActionName().c_str());
                    client.reset();
                }
                return client;
            }
    };

};





#endif // TASK_ACTION_GENERIC_H
