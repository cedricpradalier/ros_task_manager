
#include <std_msgs/Header.h>
#include "task_manager_lib/TaskClient.h"
#include "task_manager_lib/StartTask.h"
#include "task_manager_lib/StopTask.h"
#include "task_manager_lib/GetTaskList.h"
#include "task_manager_lib/GetAllTaskStatus.h"
#include "task_manager_msgs/TaskStatus.h"
#include <dynamic_reconfigure/config_tools.h>

using namespace task_manager_lib;

TaskClient::TaskClient(const std::string & node, ros::NodeHandle & nh) : spinner(1)
{
    startTaskClt = nh.serviceClient<task_manager_lib::StartTask>(node+"/start_task");
    stopTaskClt = nh.serviceClient<task_manager_lib::StopTask>(node+"/stop_task");
    getTaskListClt = nh.serviceClient<task_manager_lib::GetTaskList>(node+"/get_all_tasks");
    getAllTaskStatusClt = nh.serviceClient<task_manager_lib::GetAllTaskStatus>(node+"/get_all_status");
    updateAllStatus();

    keepAlivePub = nh.advertise<std_msgs::Header>(node+"/keep_alive",1);
    statusSub = nh.subscribe(node+"/status",0,&TaskClient::statusCallback,this);
    updateTaskList();
    updateAllStatus();
    spinner.start();
    keepAlive = false;
    keepAliveTimer = nh.createTimer(ros::Duration(0.250), &TaskClient::timerCallback, this);
    ros::Duration(0.5).sleep();
}

TaskClient::~TaskClient()
{
}

void TaskClient::timerCallback(const ros::TimerEvent &) {
    if (keepAlive) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        keepAlivePub.publish(header);
    }
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
    // ROS_INFO("Task %d %s: status %s '%s'",td.id, td.name.c_str(),taskStatusToString(td.status),td.statusString.c_str());

    boost::unique_lock<boost::mutex> lock(mutex);
    taskStatus[td.id] = td;

    std::vector<unsigned int> to_delete;
    for (StatusMap::iterator it=taskStatus.begin();it!=taskStatus.end();it++) {
        if (!(it->second.status & task_manager_msgs::TaskStatus::TASK_TERMINATED)) continue;
        if ((ts.status_time - it->second.statusTime).toSec() > 2.0) {
            to_delete.push_back(it->first);
        }
    }
    for (unsigned int i=0;i<to_delete.size();i++) {
        // ROS_INFO("Erasing task %d",to_delete[i]);
        taskStatus.erase(to_delete[i]);
    }
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
    boost::unique_lock<boost::mutex> lock(mutex);
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
    TaskParameters tp(tprm);
    tp.setParameter("foreground",foreground);
    tp.setParameter("task_period",period);
    srv.request.config = (dynamic_reconfigure::Config)tp;

    if (startTaskClt.call(srv)) {
        keepAlive = true;
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
        keepAlive = true;
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
    srv.request.id = -1;
    if (stopTaskClt.call(srv)) {
        // Would this make sense? Not sure given that we may idle while a task
        // is running in the background
        // keepAlive = false;
        return TaskScheduler::TaskId(srv.response.id);
    } else {
        ROS_ERROR("Failed to call service start_task");
        return -1;
    }
}

bool TaskClient::waitTask(TaskScheduler::TaskId tid)
{
	StatusMap::const_iterator it;
    bool finished = false;
    bool result = false;
	while (ros::ok() && !finished) {
        // TODO: blocking wait
        {
            boost::unique_lock<boost::mutex> lock(mutex);
            const StatusMap & sm = getStatusMap();
            it = sm.find(tid);
            if (it == sm.end()) {
                finished = true;
            } else if (it->second.status & task_manager_msgs::TaskStatus::TASK_TERMINATED) {
                finished = result = true;
            } else if (it->second.status > task_manager_msgs::TaskStatus::TASK_COMPLETED) {
                // Anything greater than completed is a failure situation
                finished = true;
            }
        }
#ifdef LINUX
		usleep(50000);
#endif
#ifdef WIN32
		Sleep(50);
#endif
	}
	return result;
}

void TaskClient::updateAllStatus()
{
    task_manager_lib::GetAllTaskStatus srv;
    if (!getAllTaskStatusClt.call(srv)) {
        ROS_ERROR("Failed to call service get_task_status");
        return;
    }
    boost::unique_lock<boost::mutex> lock(mutex);
    taskStatus.clear();
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

