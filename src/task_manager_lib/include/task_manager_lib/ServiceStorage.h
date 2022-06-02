#ifndef SERVICE_STORAGE_H
#define SERVICE_STORAGE_H

#include <memory>
#include <mutex>
#include <map>
#include <ros/ros.h>

namespace task_manager_lib {

    typedef std::shared_ptr<ros::ServiceClient> ServiceClientPtr;
    typedef std::map<std::string,ServiceClientPtr> ServiceMap;

    class ServiceStorage {
        private:

            ros::NodeHandle service_storage_nh;
            ServiceMap serviceMap;
            mutable std::mutex serviceMapMtx;


        public:
            ServiceStorage(ros::NodeHandle & nh) : service_storage_nh(nh) {}

            bool hasService(const std::string & s) const {
                const std::lock_guard<std::mutex> lock(serviceMapMtx);
                return serviceMap.find(s) != serviceMap.end();
            }

            ServiceClientPtr getService(const std::string & s);

            template <class srv>
                ServiceClientPtr registerServiceClient(const std::string & s, bool replace=false) {
                    const std::lock_guard<std::mutex> lock(serviceMapMtx);
                    ServiceMap::iterator it = serviceMap.find(s);
                    if (replace || (it == serviceMap.end())) {
                        ServiceClientPtr clientp(new ros::ServiceClient);
                        *clientp = service_storage_nh.serviceClient<srv>(s);
                        serviceMap[s] = clientp;
                        return clientp;
                    } else {
                        return it->second;
                    }

                }

            void registerServiceClient(const std::string & s, ServiceClientPtr client) ;
            void registerServiceClient(ServiceClientPtr client) ;

    };

};


#endif // SERVICE_STORAGE_H
