
#include "task_manager_lib/TaskClient.h"
#include "task_manager_lib/StartTask.h"
#include "task_manager_lib/StopTask.h"
#include "task_manager_lib/GetTaskList.h"
#include "task_manager_lib/GetAllTaskStatus.h"
#include "task_manager_msgs/TaskStatus.h"
#include <dynamic_reconfigure/config_tools.h>

TaskClient::TaskClient(const std::string & node, ros::NodeHandle & nh)
{
    startTaskClt = nh.serviceClient<task_manager_lib::StartTask>(node+"/start_task");
    stopTaskClt = nh.serviceClient<task_manager_lib::StopTask>(node+"/stop_task");
    getTaskListClt = nh.serviceClient<task_manager_lib::GetTaskList>(node+"/get_task_list");
    getAllTaskStatusClt = nh.serviceClient<task_manager_lib::GetAllTaskStatus>(node+"/get_all_status");
    updateAllStatus();

    statusSub = nh.subscribe(node+"/status",1,&TaskClient::statusCallback,this);
}

TaskClient::~TaskClient()
{
}


void TaskClient::statusCallback(const task_manager_msgs::TaskStatus::ConstPtr& msg) 
{
    const task_manager_msgs::TaskStatus & ts = *msg;
    TaskState td;
    td.id = ts.id;
    td.name = ts.name;
    td.status = (TaskIndicator)(ts.status & TASK_STATUS_MASK);
    td.foreground = (ts.status & TASK_FOREGROUND)?true:false; 
    td.statusString = ts.status_string;
    td.statusTime = ts.status_time;
    taskStatus[td.id] = td;
}


int TaskClient::updateTaskList()
{
    task_manager_lib::GetTaskList srv;
    if (getTaskListClt.call(srv)) {
        taskList = srv.response.tlist;
    } else {
        ROS_ERROR("Failed to call service get_task_list");
        return -1;
    }
	return 0;
}

const std::vector<task_manager_msgs::TaskDescription> & TaskClient::getTaskList() const
{
	return taskList;
}

void TaskClient::printTaskList() const
{
	unsigned int i;
	for (i=0;i<taskList.size();i++) {
		printf("Task % 3d: %-24s : %s\n",
				i, taskList[i].name.c_str(),
				taskList[i].description.c_str());
	}
}

void TaskClient::printStatusMap() const
{
	StatusMap::const_iterator it;
	for (it = taskStatus.begin();it != taskStatus.end(); it++) {
		printf("Task % 3d: %f %-12s %c %s:%s\n",
				it->second.id,
				it->second.statusTime.toSec(),
				it->second.name.c_str(),
				it->second.foreground?'F':'B',
				taskStatusToString(it->second.status),
				it->second.statusString.c_str());
	}
}


TaskScheduler::TaskId TaskClient::startTask(const std::string & taskname, 
		bool foreground, double period,
		const TaskParameters & tprm)
{
    task_manager_lib::StartTask srv;
    srv.request.name = taskname;
    dynamic_reconfigure::ConfigTools::appendParameter(srv.request.config,"main_task",foreground);
    dynamic_reconfigure::ConfigTools::appendParameter(srv.request.config,"task_period",period);
    dynamic_reconfigure::ConfigTools::appendParameter(srv.request.config,"main_task",foreground);

    if (startTaskClt.call(srv)) {
        return TaskScheduler::TaskId(srv.response.id);
    } else {
        ROS_ERROR("Failed to call service start_task");
        return -1;
    }
}

TaskScheduler::TaskId TaskClient::startTask(const std::string & taskname, 
		const TaskParameters & tprm)
{
    task_manager_lib::StartTask srv;
    srv.request.name = taskname;
    srv.request.config = tprm;
    if (startTaskClt.call(srv)) {
        return TaskScheduler::TaskId(srv.response.id);
    } else {
        ROS_ERROR("Failed to call service start_task");
        return -1;
    }
}

bool TaskClient::startTaskAndWait(const std::string & taskname, 
		const TaskParameters & tp) 
{
	TaskScheduler::TaskId tid = startTask(taskname,tp);
	return waitTask(tid);
}

int TaskClient::idle()
{
    task_manager_lib::StopTask srv;
    if (stopTaskClt.call(srv)) {
        return TaskScheduler::TaskId(srv.response.id);
    } else {
        ROS_ERROR("Failed to call service start_task");
        return -1;
    }
}

bool TaskClient::waitTask(TaskScheduler::TaskId tid)
{
	StatusMap::const_iterator it;
	while (1) {
        // TODO: blocking wait
		const StatusMap & sm = getStatusMap();
		it = sm.find(tid);
		if (it == sm.end()) {
			return false;
		}
		if (it->second.status == task_manager_msgs::TaskStatus::TASK_TERMINATED) {
			return true;
		}
		// Anything greater than terminated is a failure situation
		if (it->second.status > task_manager_msgs::TaskStatus::TASK_TERMINATED) {
			return false;
		}
#ifdef LINUX
		usleep(50000);
#endif
#ifdef WIN32
		Sleep(50);
#endif
	}
	return false;
}

void TaskClient::updateAllStatus()
{
    task_manager_lib::GetAllTaskStatus srv;
    if (!getAllTaskStatusClt.call(srv)) {
        ROS_ERROR("Failed to call service get_task_status");
        return;
    }
	for (unsigned int i=0;i<srv.response.running_tasks.size();i++) {
        const task_manager_msgs::TaskStatus & ts = srv.response.running_tasks[i];
		TaskState td;
		td.id = ts.id;
        td.name = ts.name;
        td.status = (TaskIndicator)(ts.status & TASK_STATUS_MASK);
		td.foreground = (ts.status & TASK_FOREGROUND)?true:false; 
		td.statusString = ts.status_string;
		td.statusTime = ts.status_time;
		taskStatus[td.id] = td;
	}

	for (unsigned int i=0;i<srv.response.zombie_tasks.size();i++) {
        const task_manager_msgs::TaskStatus & ts = srv.response.running_tasks[i];
		TaskState td;
		td.id = ts.id;
        td.name = ts.name;
        td.status = (TaskIndicator)(ts.status & TASK_STATUS_MASK);
		td.foreground = (ts.status & TASK_FOREGROUND)?true:false; 
		td.statusString = ts.status_string;
		td.statusTime = ts.status_time;
		taskStatus[td.id] = td;
	}

}

