#ifndef TASK_ACTION_GENERIC_H
#define TASK_ACTION_GENERIC_H


#include <task_manager_msgs/TaskStatus.h>
#include <task_manager_lib/TaskDefinition.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/action_definition.h>


namespace task_manager_action {
    
    template <class Action, class TaskConfig,class TaskEnvironment>
    class TaskActionGeneric : public task_manager_lib::TaskInstance<TaskConfig,TaskEnvironment>
    {
        protected:
            ACTION_DEFINITION(Action)

            typedef actionlib::SimpleActionClient<Action> Client;
            Client *client;

            // This must be declared by inheriting class
            virtual const std::string & getActionName() const = 0;
            virtual void buildActionGoal(Goal& goal) const = 0;

            // This can be overwritten if needed
            virtual void handleResult(ResultConstPtr result) {};

        public:
            // Basic constructor, receives the environment and ignore it.
            TaskActionGeneric(task_manager_lib::TaskDefinitionPtr def, 
                    task_manager_lib::TaskEnvironmentPtr ev) :
                task_manager_lib::TaskInstance<TaskConfig,TaskEnvironment>(def,ev), client(NULL) { }
            virtual ~TaskActionGeneric() { 
                delete client; 
            };

            /// Record the starting time
            virtual task_manager_lib::TaskIndicator initialise() {
                Goal goal;
                client = new Client(getActionName(),true);
                if (!client->waitForServer(ros::Duration(5.0))) {
                    ROS_ERROR("Action server %s did not reply after 5s, aborting task",getActionName().c_str());
                    delete client; client = NULL;
                    return task_manager_msgs::TaskStatus::TASK_INITIALISATION_FAILED;
                }
                buildActionGoal(goal);
                client->sendGoal(goal);
                return task_manager_msgs::TaskStatus::TASK_INITIALISED;
            }

            virtual task_manager_lib::TaskIndicator iterate() {
                switch (client->getState().state_) {
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        this->handleResult(client->getResult());
                        return task_manager_msgs::TaskStatus::TASK_COMPLETED;
                    case actionlib::SimpleClientGoalState::PENDING:
                    case actionlib::SimpleClientGoalState::ACTIVE:
                        return task_manager_msgs::TaskStatus::TASK_RUNNING;
                    default:
                        return task_manager_msgs::TaskStatus::TASK_FAILED;
                }
            }

            virtual task_manager_lib::TaskIndicator terminate() {
                if (client) {
                    switch (client->getState().state_) {
                        case actionlib::SimpleClientGoalState::PENDING:
                        case actionlib::SimpleClientGoalState::ACTIVE:
                            client->cancelGoal();
                            break;
                        default:
                            break;
                    }
                }
                delete client;
                client = NULL;
                return task_manager_msgs::TaskStatus::TASK_TERMINATED;
            }

    };

};





#endif // TASK_ACTION_GENERIC_H
