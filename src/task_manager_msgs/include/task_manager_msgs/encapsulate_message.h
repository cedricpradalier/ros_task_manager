#ifndef TASK_MANAGER_MSGS_ENCAPSULATE
#define TASK_MANAGER_MSGS_ENCAPSULATE

#include <ros/ros.h>
#include <task_manager_msgs/EncapsulatedMessage.h>
#include <iostream>     // std::cout, std::ostream, std::hex
#include <sstream>      // std::stringbuf

namespace task_manager_msgs {
    template <class Msg>
        bool encapsulate(EncapsulatedMessage & E, const Msg & m) {
            E.type = ros::message_traits::DataType<Msg>::value();
            E.md5sum = ros::message_traits::MD5Sum<Msg>::value();
            uint32_t length = ros::serialization::serializationLength(m);
            uint8_t data[length];
            ros::serialization::OStream os(data,length);
            ros::serialization::serialize(os,m);
            E.data.resize(length);
            std::copy(data,data+length,E.data.begin());
            return true;
        }

    template <class Msg>
        bool decapsulate(Msg & m, const EncapsulatedMessage & E) {
            // ROS_INFO("Decapsulate: E.type '%s' sum '%s'",
            //         E.type.c_str(),E.md5sum.c_str());
            if (E.type != ros::message_traits::DataType<Msg>::value()) {
                ROS_ERROR("Decapsulate: invalid msg type");
                return false;
            }
            if (E.md5sum != ros::message_traits::MD5Sum<Msg>::value()) {
                ROS_ERROR("Decapsulate: invalid MD5 sum");
                return false;
            }
            uint8_t data[E.data.size()];
            std::copy(E.data.begin(),E.data.end(),data);
            ros::serialization::IStream is(data,E.data.size());
            ros::serialization::deserialize(is,m);
            return true;
        }

};


#endif // TASK_MANAGER_MSGS_ENCAPSULATE
