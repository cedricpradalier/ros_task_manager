
#include "task_manager_lib/ServiceStorage.h"

using namespace task_manager_lib;


ServiceClientPtr ServiceStorage::getService(const std::string & s) {
	const std::lock_guard<std::mutex> lock(serviceMapMtx);
	ServiceMap::iterator it = serviceMap.find(s);
	if (it == serviceMap.end()) {
		return ServiceClientPtr();
	} else {
		return it->second;
	}
}

void ServiceStorage::registerServiceClient(const std::string & s, ServiceClientPtr client) {
	assert(client);
	const std::lock_guard<std::mutex> lock(serviceMapMtx);
	serviceMap[s]=client;
}

void ServiceStorage::registerServiceClient(ServiceClientPtr client) {
	assert(client);
	const std::lock_guard<std::mutex> lock(serviceMapMtx);
	serviceMap[client->getService()]=client;
}




