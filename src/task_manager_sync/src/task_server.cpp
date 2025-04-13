#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_sync/TaskServerSync.h"

#include "task_manager_sync/TaskEnvironmentSync.h"


using namespace task_manager_sync;
using namespace task_manager_lib;

class TaskServer : public TaskServerSync {
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerSync(_env) {
            start();
        }

};

class TestSyncEnv: public task_manager_sync::TaskEnvironmentSync
{
    public:
        TestSyncEnv(std::shared_ptr<rclcpp::Node> node, const std::string & name) 
            : task_manager_sync::TaskEnvironmentSync(node,name) {}
        ~TestSyncEnv() {}

        DECLARE_ENV_CHECKSUM;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);//init ros
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("task_server_sync");
    std::string partner_name = "partner";
    node->declare_parameter("my_name", partner_name);
    partner_name = node->get_parameter("my_name").get_parameter_value().get<std::string>();

    TaskEnvironmentSyncPtr env(new TestSyncEnv(node,partner_name));
    env->addSyncSource("partner1");
    env->addSyncSource("partner2");
    TaskServer ts(env);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
