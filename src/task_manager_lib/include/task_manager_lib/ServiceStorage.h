#ifndef SERVICE_STORAGE_H
#define SERVICE_STORAGE_H

#include <memory>
#include <mutex>
#include <map>
#include <rclcpp/rclcpp.hpp>

namespace task_manager_lib {

    typedef std::shared_ptr<rclcpp::ClientBase> ServiceClientPtr;
    typedef std::map<std::string,ServiceClientPtr> ServiceMap;

    class ServiceStorage {
        private:

            std::shared_ptr<rclcpp::Node> node;
            ServiceMap serviceMap;
            mutable std::mutex serviceMapMtx;


        public:
            ServiceStorage(std::shared_ptr<rclcpp::Node> & nh) : node(nh) {}

            bool hasService(const std::string & s) const {
                const std::lock_guard<std::mutex> lock(serviceMapMtx);
                return serviceMap.find(s) != serviceMap.end();
            }

            template <class srv> 
                typename rclcpp::Client<srv>::SharedPtr getService(const std::string & s) {
                    const std::lock_guard<std::mutex> lock(serviceMapMtx);
                    ServiceMap::iterator it = serviceMap.find(s);
                    typename rclcpp::Client<srv>::SharedPtr e;
                    if (it != serviceMap.end()) {
                        e = std::dynamic_pointer_cast<rclcpp::Client<srv>,rclcpp::ClientBase>(it->second);
                    }
                    return e;
                }


            template <class srv>
                typename rclcpp::Client<srv>::SharedPtr registerServiceClient(const std::string & s, bool replace=false) {
                    const std::lock_guard<std::mutex> lock(serviceMapMtx);
                    ServiceMap::iterator it = serviceMap.find(s);
                    if (replace || (it == serviceMap.end())) {
                        RCLCPP_INFO(node->get_logger(),"Creating new service client for '%s'", s.c_str());
                        typename rclcpp::Client<srv>::SharedPtr clientp = node->create_client<srv>(s);
                        serviceMap[s] = clientp;
                        return clientp;
                    } else {
                        RCLCPP_INFO(node->get_logger(),"Reusing service client for '%s'", s.c_str());
                        typename rclcpp::Client<srv>::SharedPtr e =
                            std::dynamic_pointer_cast<rclcpp::Client<srv>,rclcpp::ClientBase>(it->second);
                        return e;
                    }

                }

            template <class srv>
                void registerServiceClient(const std::string & s, typename rclcpp::Client<srv>::SharedPtr client) {
                    assert(client);
                    const std::lock_guard<std::mutex> lock(serviceMapMtx);
                    serviceMap[s]=client;
                }

            template <class srv>
                void registerServiceClient(typename rclcpp::Client<srv>::SharedPtr client) {
                    assert(client);
                    const std::lock_guard<std::mutex> lock(serviceMapMtx);
                    serviceMap[client->get_service_name()]=client;
                }
            

    };

}


#endif // SERVICE_STORAGE_H
