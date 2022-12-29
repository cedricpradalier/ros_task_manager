#ifndef TASK_MANAGER_MSGS_ENCAPSULATE
#define TASK_MANAGER_MSGS_ENCAPSULATE

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include <task_manager_msgs/msg/encapsulated_message.hpp>

namespace task_manager_msgs {
    template <class Msg>
        bool encapsulate(task_manager_msgs::msg::EncapsulatedMessage & E, const Msg & m) {
            E.type = rosidl_generator_traits::name<Msg>();
            E.md5sum = "";
            rclcpp::Serialization<Msg> serializer;
            rclcpp::SerializedMessage serialized_msg;
            serializer.serialize_message(&m,&serialized_msg);
            uint32_t length = serialized_msg.size();
            uint8_t *data = serialized_msg.get_rcl_serialized_message().buffer;
            E.data.clear();
            for (uint8_t *it=data;it!=data+length;it++) {
                E.data.push_back(*it);
            }
            return true;
        }

    template <class Msg>
        bool decapsulate(Msg & m, const task_manager_msgs::msg::EncapsulatedMessage & E) {
            // ROS_INFO("Decapsulate: E.type '%s' sum '%s'",
            //         E.type.c_str(),E.md5sum.c_str());
            if (E.type != rosidl_generator_traits::name<Msg>()) {
                printf("Decapsulate: invalid msg type");
                return false;
            }
#if 0 // disable for ROS2
            if (E.md5sum != ros::message_traits::MD5Sum<Msg>::value()) {
                ROS_ERROR("Decapsulate: invalid MD5 sum");
                return false;
            }
#endif
            rclcpp::SerializedMessage serialized_msg(E.data.size());
            serialized_msg.get_rcl_serialized_message().buffer_length=E.data.size();
            uint8_t *buffer = serialized_msg.get_rcl_serialized_message().buffer;
            for (std::string::const_iterator it=E.data.begin();it!=E.data.end();it++) {
                *buffer++ = *it;
            }
            rclcpp::Serialization<Msg> serializer;
            serializer.deserialize_message(&serialized_msg,&m);
            return true;
        }

}


#endif // TASK_MANAGER_MSGS_ENCAPSULATE
