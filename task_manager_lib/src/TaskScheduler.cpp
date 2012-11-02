#include <dirent.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <ros/ros.h>

#include "task_manager_lib/DynamicTask.h"
#include "task_manager_lib/TaskScheduler.h"
#include <dynamic_reconfigure/config_tools.h>

using namespace std;
using namespace task_manager_lib;

#define PRINTF(level,X...) if (level <= (signed)debug) printf(X)

unsigned int TaskScheduler::ThreadParameters::gtpid = 0;
unsigned int TaskScheduler::debug = 0;
const double TaskScheduler::DELETE_TIMEOUT=2.0;
const double TaskScheduler::IDLE_TIMEOUT=0.5;

TaskScheduler::ThreadParameters::ThreadParameters(ros::Publisher pub, TaskScheduler *ts, 
		boost::shared_ptr<TaskDefinition> td, double tperiod) : statusPub(pub)
{
	gtpid += 1;
	tpid = gtpid;
	tid = 0;
	task = td;
	task->resetStatus();
	that = ts;
	period = tperiod;
	foreground = true;
	pthread_cond_init(&task_condition,NULL);
	pthread_mutex_init(&task_mutex,NULL);
	pthread_cond_init(&aperiodic_task_condition,NULL);
	pthread_mutex_init(&aperiodic_task_mutex,NULL);
}


TaskScheduler::ThreadParameters::ThreadParameters(
		const TaskScheduler::ThreadParameters &tp)
{
	tpid = tp.tpid;
	tid = tp.tid;
	task = tp.task;
	that = tp.that;
	period = tp.period;
	status = tp.status;
	foreground = true;
	statusTime = tp.statusTime;
	statusString = tp.statusString;
    statusPub = tp.statusPub;
	pthread_cond_init(&aperiodic_task_condition,NULL);
	pthread_mutex_init(&aperiodic_task_mutex,NULL);
	pthread_cond_init(&task_condition,NULL);
	pthread_mutex_init(&task_mutex,NULL);
}

TaskScheduler::ThreadParameters::~ThreadParameters()
{
	pthread_cond_destroy(&task_condition);
	pthread_mutex_destroy(&task_mutex);
	pthread_cond_destroy(&aperiodic_task_condition);
	pthread_mutex_destroy(&aperiodic_task_mutex);
}

TaskScheduler::TaskScheduler(ros::NodeHandle & nh, boost::shared_ptr<TaskDefinition> tidle, double deftPeriod)
{
	aqid = 0;
	runScheduler = false;

	idle = tidle;
	idleTimeout = 0.001;

	tasks["idle"] = idle;
	defaultPeriod = deftPeriod;
	startingTime = ros::Time::now().toSec();

	mainThread.reset();
	pthread_mutex_init(&scheduler_mutex,NULL);
	pthread_cond_init(&scheduler_condition,NULL);
	pthread_mutex_init(&aqMutex,NULL);
	pthread_cond_init(&aqCond,NULL);

	ROS_INFO("Task scheduler created: debug %d\n",debug);

    n = nh;
    startTaskSrv = nh.advertiseService("start_task", &TaskScheduler::startTask,this);
    stopTaskSrv = nh.advertiseService("stop_task", &TaskScheduler::stopTask,this);
    getTaskListSrv = nh.advertiseService("get_all_tasks", &TaskScheduler::getTaskList,this);
    getTaskListLightSrv =nh.advertiseService("get_all_tasks_light", &TaskScheduler::getTaskListLight,this);
    getAllTaskStatusSrv = nh.advertiseService("get_all_status", &TaskScheduler::getAllTaskStatus,this);
    statusPub = nh.advertise<task_manager_msgs::TaskStatus>("status",20);
    keepAliveSub = nh.subscribe("keep_alive",1,&TaskScheduler::keepAliveCallback,this);
    lastKeepAlive = ros::Time::now();
}

void TaskScheduler::keepAliveCallback(const std_msgs::Header::ConstPtr& msg) 
{
    lastKeepAlive = ros::Time::now();
}

TaskScheduler::~TaskScheduler()
{
	terminateAllTasks();
	stopScheduler();

	zombieThreads.clear();
	tasks.clear();
	pthread_mutex_destroy(&scheduler_mutex);
	pthread_cond_destroy(&scheduler_condition);
	pthread_mutex_destroy(&aqMutex);
	pthread_cond_destroy(&aqCond);
}

bool TaskScheduler::startTask(task_manager_lib::StartTask::Request  &req,
         task_manager_lib::StartTask::Response &res )
{
    lastKeepAlive = ros::Time::now();
    TaskId id = launchTask(req.name,TaskParameters(req.config));
    res.id = id;
    return true;
}

bool TaskScheduler::stopTask(task_manager_lib::StopTask::Request  &req,
         task_manager_lib::StopTask::Response &res )
{
    lastKeepAlive = ros::Time::now();
    if (req.id == -1) {
        TaskId id = launchIdleTask();
        res.id = id;
    } else {
        TaskSet::iterator it = runningThreads.find(req.id);
        if (it != runningThreads.end()) {
            terminateTask(it->second);
        }
        res.id = 0;
    }
    return true;
}

bool TaskScheduler::getTaskList(task_manager_lib::GetTaskList::Request  &req,
         task_manager_lib::GetTaskList::Response &res )
{
    lastKeepAlive = ros::Time::now();
    generateTaskList(res.tlist);
    return true;
}

bool TaskScheduler::getTaskListLight(task_manager_lib::GetTaskListLight::Request  &req, task_manager_lib::GetTaskListLight::Response &res )
{
	task_manager_lib::GetTaskList::Response res1;
	generateTaskList(res1.tlist);
	generateTaskListLight(res1.tlist,res.tlist);
	return true;
}



bool TaskScheduler::getAllTaskStatus(task_manager_lib::GetAllTaskStatus::Request  &req,
         task_manager_lib::GetAllTaskStatus::Response &res )
{
    lastKeepAlive = ros::Time::now();
    generateTaskStatus(res.running_tasks,res.zombie_tasks);
    return true;
}


void TaskScheduler::lockScheduler()
{
	pthread_mutex_lock(&scheduler_mutex);
}

void TaskScheduler::unlockScheduler()
{
	pthread_mutex_unlock(&scheduler_mutex);
}


int TaskScheduler::terminateAllTasks()
{
	TaskSet copy = runningThreads;
	TaskSet::iterator it;
	PRINTF(1,"Terminating all tasks\n");
	for (it = copy.begin();it!=copy.end();it++) {
		// delete pointer and empty the list of running tasks
		terminateTask(it->second);
	}
	runningThreads.clear();
	mainThread.reset();
	return 0;
}

void TaskScheduler::addTask(boost::shared_ptr<TaskDefinition> td) 
{
	tasks.insert(std::pair< std::string,boost::shared_ptr<TaskDefinition> >(td->getName(),td));
}

void TaskScheduler::loadTask(const std::string & filename, boost::shared_ptr<TaskEnvironment> env)
{
    boost::shared_ptr<TaskDefinition> td(new DynamicTask(filename, env));

	addTask(td);
}

void TaskScheduler::configureTasks()
{
	TaskDirectory::const_iterator tit;
	for (tit = tasks.begin();tit!=tasks.end();tit++) {
		// Try loading from the parameter server / launch file
        TaskParameters tp = tit->second->getParametersFromServer(n);
        // printf("TS %s param from server\n",tit->second->getName().c_str());
        // tp.print();
        std::string rename;
        if (tp.getParameter("task_rename",rename) && !rename.empty()) {
            tit->second->setName(rename);
        }
		tit->second->doConfigure(tp);
	}
}

#define DLL_EXT "so"

static int dllfilter(const struct dirent * d) {
	std::string name(d->d_name);
	unsigned int pos = name.find_last_of(".");
	if (pos != std::string::npos) {
		std::string suffix = name.substr(pos+1,std::string::npos);
		return (suffix == DLL_EXT)?1:0;
	} 
	return 0;
}

void TaskScheduler::loadAllTasks(const std::string & dirname, 
		boost::shared_ptr<TaskEnvironment> env)
{
    struct dirent **namelist;
    int n;
    n = scandir(dirname.c_str(), &namelist, dllfilter, alphasort);
    if (n < 0)
        perror("scandir");
    else {
        while(n--) {
			std::string fname = dirname + "/" + namelist[n]->d_name;
            loadTask(fname,env);
            free(namelist[n]);
        }
        free(namelist);
    }
}


void TaskScheduler::cleanitup(void * arg)
{
    boost::shared_ptr<ThreadParameters> tp = *(boost::shared_ptr<ThreadParameters>*)arg;
	TaskScheduler *that = tp->that;
	that->cleanupTask(tp);
}

void * TaskScheduler::thread_func(void * arg)
{
    boost::shared_ptr<ThreadParameters> tp = *(boost::shared_ptr<ThreadParameters>*)arg;
	TaskScheduler *that = tp->that;
	pthread_cleanup_push(cleanitup,&tp);
	that->runTask(tp);
	pthread_cleanup_pop(1);
	return NULL;
}

TaskScheduler::TaskId TaskScheduler::launchIdleTask()
{
	double period = defaultPeriod;

	if (mainThread) {
		if (mainThread->task == idle) {
			return mainThread->tpid;
		} else {
			terminateTask(mainThread);
		}
	}

	// Finally create the thread responsible for running the task
	PRINTF(3,"lit:Locking\n");
	lockScheduler();
	PRINTF(3,"lit:Locked\n");
	mainThread = boost::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, idle, period));
	mainThread->foreground = true;
	runningThreads[mainThread->tpid] = mainThread;
	if (debug>=3) printTaskSet("After launch idle",runningThreads);
	unlockScheduler();
	PRINTF(3,"lit:Unlocked\n");

	if (pthread_create(&mainThread->tid,NULL,thread_func,&mainThread)) {
		return 0;
	}
	return mainThread->tpid;
}

TaskScheduler::TaskId TaskScheduler::launchTask(boost::shared_ptr<ThreadParameters> tp)
{

	if (tp->foreground) {
		terminateTask(mainThread);
	}

	PRINTF(3,"lt:Locking\n");
	lockScheduler();
	PRINTF(3,"lt:Locked\n");

	if (tp->foreground) {
		mainThread = tp;
	}
	runningThreads[tp->tpid] = tp;
	if (debug>=3) printTaskSet("After launch",runningThreads);
	unlockScheduler();
	PRINTF(3,"lt:Unlocked\n");

	pthread_mutex_lock(&tp->task_mutex);
	if (pthread_create(&tp->tid,NULL,thread_func,&tp)) {
		// there is a risk to check here
		pthread_cond_broadcast(&tp->task_condition);
		pthread_mutex_unlock(&tp->task_mutex);
		if (tp->foreground) {
			launchIdleTask();
		}
		enqueueAction(ros::Time::now()+ros::Duration(DELETE_TIMEOUT),DELETE_TASK,tp);
		return 0;
	} else {
		if (tp->foreground) {
			removeConditionalIdle();
		}
	}
	return tp->tpid;
}

TaskScheduler::TaskId TaskScheduler::launchTask(const std::string & taskname, 
		const TaskParameters & tp)
{
	bool mainTask = true;
	double period = defaultPeriod;
	TaskDirectory::const_iterator tdit;
	tdit = tasks.find(taskname);
	if (tdit==tasks.end()) {
		ROS_ERROR("Impossible to find task '%s'\n",taskname.c_str());
		return -1;
	}

	// See if some runtime period has been defined in the parameters
    if (!tp.getParameter("task_period",period)) {
		ROS_ERROR("Missing required parameter task_period\n");
		return -1;
	}
    tp.getParameter("main_task",mainTask); // ignore return

	// Finally create the thread responsible for running the task
    boost::shared_ptr<ThreadParameters> tparam =
        boost::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, tdit->second, period));
	tparam->params = tp;
	tparam->foreground = mainTask;
	tparam->running = false;
	
	PRINTF(3,"lt:Locking\n");
	pthread_mutex_lock(&tparam->task_mutex);
	PRINTF(3,"lt:Locked\n");
	
	enqueueAction(START_TASK,tparam);
	
	PRINTF(3,"lt:Wait condition\n");
	pthread_cond_wait(&tparam->task_condition,&tparam->task_mutex);
	PRINTF(3,"lt:Locked\n");
	pthread_mutex_unlock(&tparam->task_mutex);
	PRINTF(3,"lt:Unlocked\n");

	return tparam->tpid;
}

void * TaskScheduler::runAperiodicTask(void *arg)
{
    boost::shared_ptr<ThreadParameters> tp = *(boost::shared_ptr<ThreadParameters>*)arg;

	// Just a barrier...
	pthread_mutex_lock(&tp->aperiodic_task_mutex);
	pthread_mutex_unlock(&tp->aperiodic_task_mutex);

	tp->task->doIterate();
	// WARNING: this might be misinterpreted. Check this.
	pthread_cond_broadcast(&tp->task_condition);
	return NULL;
}

int TaskScheduler::runTask(boost::shared_ptr<ThreadParameters> tp)
{
    double tstart = ros::Time::now().toSec();
	pthread_cond_broadcast(&tp->task_condition);
	pthread_mutex_unlock(&tp->task_mutex);
    tp->updateStatus(ros::Time::now());

	tp->running = true;
	try {
		tp->task->doInitialise(tp->params);
		tp->updateStatus(ros::Time::now());
	} catch (const std::exception & e) {
		tp->task->debug("Exception %s\n",e.what());
		tp->updateStatus(ros::Time::now());
		return -1;
	}
	tp->running = true;
	ROS_INFO("Running task '%s' at period %f main %d timeout %f",tp->task->getName().c_str(),tp->period,(tp==mainThread),tp->task->getTimeout());

	if (tp->task->isPeriodic()) {
		PRINTF(2,"Initialisation done\n");
		while (1) {
            double t0 = ros::Time::now().toSec();
            if (mainThread && (mainThread->task!=idle) && (t0 - lastKeepAlive.toSec() > 1.0)) {
				tp->task->debug("KEEPALIVE failed\n");
				tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",ros::Time(t0));
				break;
            }

			if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
				tp->task->debug("TIMEOUT\n");
				tp->setStatus(task_manager_msgs::TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",ros::Time(t0));
				break;
			}

            double t1 = t0;
			try {
				// tp->task->debug("Iterating...\n");
				tp->task->doIterate();
				tp->updateStatus(now());
			} catch (const std::exception & e) {
				tp->task->debug("Exception %s\n",e.what());
				tp->updateStatus(now());
				return -1;
			}
			if (tp->status != task_manager_msgs::TaskStatus::TASK_RUNNING) {
				PRINTF(2,"Task not running anymore\n");
				break;
			}
			usleep((unsigned int)(std::max(1e-3,(tp->period - (t1-t0)))*1e6));
		}
	} else {
		bool first = true;
		pthread_t id;
		pthread_mutex_lock(&tp->aperiodic_task_mutex);
		pthread_create(&id, NULL, runAperiodicTask, &tp);
		while (1) {
			struct timespec ts;
            double t0 = ros::Time::now().toSec();
            if (mainThread && (mainThread->task!=idle) && (t0 - lastKeepAlive.toSec() > 1.0)) {
				tp->task->debug("KEEPALIVE failed\n");
				tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",ros::Time(t0));
				break;
            }
			if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
				tp->task->debug("TIMEOUT\n");
				tp->setStatus(task_manager_msgs::TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",ros::Time(t0));
				break;
			}
            double t1 = t0;
			tp->updateStatus(ros::Time::now());
			if (!first && tp->status != task_manager_msgs::TaskStatus::TASK_RUNNING) {
				tp->task->debug("Task not running anymore\n");
				break;
			}

			double ttimeout = ros::Time::now().toSec() +
				std::max(1e-3,(tp->period - (t1-t0)));
			ts.tv_sec = (unsigned long)floor(ttimeout);
			ts.tv_nsec = (unsigned long)floor((ttimeout - ts.tv_sec)*1e9);
			pthread_cond_timedwait(&tp->aperiodic_task_condition,
					&tp->aperiodic_task_mutex,&ts);

			first = false;
		}
		pthread_mutex_unlock(&tp->aperiodic_task_mutex);
		if (tp->status != task_manager_msgs::TaskStatus::TASK_COMPLETED) {
			pthread_cancel(id);
		}
		pthread_join(id,NULL);
	}
	// tp->task->debug("Out of the loop\n");
	return 0;
}

int TaskScheduler::terminateTask(boost::shared_ptr<ThreadParameters> tp)
{
	if (!tp) return 0;
	PRINTF(1,"Terminating thread %s\n",tp->task->getName().c_str());

	pthread_cancel(tp->tid);
	pthread_join(tp->tid,NULL);
	tp->tid = 0;
	tp->running = false;

	PRINTF(2,"Thread cancelled\n");
	return 0;
}

void TaskScheduler::printTaskSet(const std::string & name, const TaskScheduler::TaskSet & ts)
{
	printf("TaskSet: %s\n",name.c_str());
	TaskSet::const_iterator it;
	for (it = ts.begin();it!=ts.end();it++) {
		printf("%d: %s\n",it->first,it->second->task->getName().c_str());
	}
	printf("-------------------\n");
}

int TaskScheduler::cleanupTask(boost::shared_ptr<ThreadParameters> tp)
{
	if (tp == NULL) return 0;
	PRINTF(1,"Cleaning up task %d:%s\n",tp->tpid,tp->task->getName().c_str());
	tp->task->doTerminate();
	PRINTF(2,"End of terminate\n");
	tp->status = task_manager_msgs::TaskStatus::TASK_TERMINATED;
	tp->statusTime = now();
	tp->statusString = "terminated";
    tp->updateStatus(now());

	lockScheduler();
	zombieThreads.insert(TaskSetItem(tp->tpid,tp));
	TaskSet::iterator tsit = runningThreads.find(tp->tpid);
	assert(tsit != runningThreads.end());
	runningThreads.erase(tsit);

	if (debug>=3) printTaskSet("Zombies at cleanup",zombieThreads);
	if (debug>=3) printTaskSet("Running at cleanup",runningThreads);


	if (tp->foreground) {
		mainThread.reset();
		if (tp->task!=idle) {
			enqueueAction(ros::Time::now()+ros::Duration(IDLE_TIMEOUT),CONDITIONALLY_IDLE,tp);
		}
	}
	enqueueAction(ros::Time::now()+ros::Duration(DELETE_TIMEOUT),DELETE_TASK,tp);
	
	// There is a risk that we trigger the execution of a new task before
	// starting idle here. In addition, we need one condition per task
	pthread_cond_broadcast(&tp->task_condition);
	// pthread_cond_broadcast(&scheduler_condition);
	unlockScheduler();

	
	return 0;
}

int TaskScheduler::deleteTask(boost::shared_ptr<ThreadParameters> tp)
{
	TaskSet::iterator it = zombieThreads.find(tp->tpid);
	assert(it != zombieThreads.end());
	zombieThreads.erase(it);
	if (tp->tid) {
		pthread_join(tp->tid,NULL);
	}
	tp.reset();
	return 0;
}

int TaskScheduler::waitTaskCompletion(TaskId id, double timeout)
{
	int cwres;
	struct timespec ts;
	double ttimeout = ros::Time::now().toSec() + timeout;
	TaskSet::iterator it;
	lockScheduler();
	if (debug>=3) printTaskSet("Zombies when waiting",zombieThreads);
	if (debug>=3) printTaskSet("Running when waiting",runningThreads);

	it = zombieThreads.find(id);
	if (it != zombieThreads.end()) {
		unlockScheduler();
		return 0;
	}
	it = runningThreads.find(id);
	if (it == runningThreads.end()) {
		ROS_ERROR("Cannot find reference to task %d\n",id);
		unlockScheduler();
		return -1;
	}
	ts.tv_sec = (unsigned long)floor(ttimeout);
	ts.tv_nsec = (unsigned long)floor((ttimeout - ts.tv_sec)*1e9);
	// cwres = pthread_cond_timedwait(&scheduler_condition,&scheduler_mutex,&ts);
	cwres = pthread_cond_timedwait(&(it->second->task_condition),&scheduler_mutex,&ts);
	// the scheduler mutex is garanteed to be locked back here
	unlockScheduler();
	return cwres;
}


void TaskScheduler::printTaskDirectory(bool with_ros) const
{
	unsigned int i = 0;
	TaskDirectory::const_iterator tit;
	if (with_ros) {
		ROS_INFO("Task Directory:");
		for (tit = tasks.begin();tit!=tasks.end();tit++) {
			ROS_INFO("%d -- %s: %s",i,tit->second->getName().c_str(),tit->second->getHelp().c_str());
			i++;
		}
	} else {
		printf("Task Directory:\n");
		for (tit = tasks.begin();tit!=tasks.end();tit++) {
			printf("%d -- %s: %s\n",i,tit->second->getName().c_str(),tit->second->getHelp().c_str());
			i++;
		}
		printf("---------------\n");
	}
}

void TaskScheduler::cleanup_action(void *arg) {
	ThreadAction *ta = (ThreadAction*)arg;
	PRINTF(2,"Action wait cancelled\n");
	ta->type = WAIT_CANCELLED;
	ta->tp.reset();
}

void TaskScheduler::removeConditionalIdle()
{
	PRINTF(3,"rci:Locking\n");
	pthread_mutex_lock(&aqMutex);
	PRINTF(3,"rci:Locked\n");

	ActionQueue::iterator it = actionQueue.begin();
	while (it != actionQueue.end()) {
		if (it->second.type == CONDITIONALLY_IDLE) {
			ActionQueue::iterator copy = it;
			it ++;
			actionQueue.erase(copy);
		} else {
			it ++;
		}
	}

	PRINTF(3,"rci:Signalling\n");
	pthread_cond_signal(&aqCond);
	pthread_mutex_unlock(&aqMutex);
	PRINTF(3,"rci:Unlocked\n");
}

TaskScheduler::ThreadAction TaskScheduler::getNextAction()
{
	// Warning: add cancel management
    ros::Time t;
	ThreadAction ta;
	ActionQueue::iterator it;
	PRINTF(3,"gna:Locking\n");
	pthread_mutex_lock(&aqMutex);
	PRINTF(3,"gna:Locked\n");
	pthread_cleanup_push(cleanup_action,&ta);
	// TODO: this must get the next action in time
	if (actionQueue.empty()) {
		PRINTF(3,"gna:Cond Wait\n");
		pthread_cond_wait(&aqCond,&aqMutex);
		PRINTF(3,"gna:Locked\n");
	}
	while (1) {
		t = ros::Time::now();
		it = actionQueue.begin();
		assert(it != actionQueue.end());
		if (runScheduler) {
			if (it->first <= t.toSec()) {
				ta = it->second;
				actionQueue.erase(it);
			} else {
				// wait for the right time or another action to be inserted
				struct timespec ts;
				double ttimeout = it->first;
				ts.tv_sec = (unsigned long)floor(ttimeout);
				ts.tv_nsec = (unsigned long)floor((ttimeout - ts.tv_sec)*1e9);
				PRINTF(3,"gna:Cond TWait\n");
				int cwres = pthread_cond_timedwait(&aqCond,&aqMutex,&ts);
				PRINTF(3,"gna:Locked\n");
				switch (cwres) {
					case ETIMEDOUT:
					case 0:
						continue;
					default:
						assert(cwres == 0);
				}
			}
		} else {
			ta = it->second;
			actionQueue.erase(it);
		}
		break;
	}
	pthread_cleanup_pop(0);
	PRINTF(3,"gna:Unlocking\n");
	pthread_mutex_unlock(&aqMutex);
	
	return ta;
}

void TaskScheduler::enqueueAction(ActionType type,boost::shared_ptr<ThreadParameters> tp)
{
	ThreadAction ta;
	PRINTF(3,"ea:Locking\n");
	pthread_mutex_lock(&aqMutex);
	PRINTF(3,"ea:Locked\n");
	// if (!runScheduler) return;
    double when = ros::Time::now().toSec();
	PRINTF(2,"Enqueueing action %.3f %s -- %s\n",when,actionString(type),
			tp?(tp->task->getName().c_str()):"none");

	ta.type = type;
	ta.tp = tp;
	actionQueue[when] = ta;
	PRINTF(3,"ea:Signalling\n");
	pthread_cond_signal(&aqCond);
	PRINTF(3,"ea:Unlocking\n");
	pthread_mutex_unlock(&aqMutex);
}

void TaskScheduler::enqueueAction(const ros::Time & when,  ActionType type,boost::shared_ptr<ThreadParameters> tp)
{
	ThreadAction ta;
	PRINTF(3,"ea:Locking\n");
	pthread_mutex_lock(&aqMutex);
	PRINTF(3,"ea:Locked\n");
	// if (!runScheduler) return;
	PRINTF(2,"Enqueing action %.3f %s -- %s\n",when.toSec(),actionString(type),
			tp?(tp->task->getName().c_str()):"none");

	ta.type = type;
	ta.tp = tp;
	actionQueue[when.toSec()] = ta;
	PRINTF(3,"ea:Signalling\n");
	pthread_cond_signal(&aqCond);
	PRINTF(3,"ea:Unlocking\n");
	pthread_mutex_unlock(&aqMutex);
}

void * TaskScheduler::scheduler_thread(void *arg)
{
	TaskScheduler *that = (TaskScheduler*)arg;
	that->runSchedulerLoop();
	return NULL;
}

const char *TaskScheduler::actionString(ActionType at)
{
	switch (at) {
		case START_IDLE_TASK:
			return "START_IDLE_TASK";
			break;
		case START_TASK:
			return "START_TASK";
			break;
		case DELETE_TASK:
			return "DELETE_TASK";
			break;
		case CONDITIONALLY_IDLE:
			return "CONDITIONALLY_IDLE";
			break;
		case WAIT_CANCELLED:
			return "WAIT_CANCELLED";
			break;
		default:
			break;
	}
	return "Unknown";
}

int TaskScheduler::runSchedulerLoop()
{
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	while (1) {
		PRINTF(2,"Waiting next action (%d)\n",actionQueue.size());
		if (!runScheduler && actionQueue.empty()) {
			break;
		}
		
		ThreadAction ta = getNextAction();
		PRINTF(2,"%f: got action ",ros::Time::now().toSec());
		PRINTF(3,"rsl:Locking\n");
		switch (ta.type) {
			case START_IDLE_TASK:
				PRINTF(2,"START_IDLE_TASK\n");
				launchIdleTask();
				break;
			case START_TASK:
				PRINTF(2,"START_TASK %s\n",ta.tp->task->getName().c_str());
				launchTask(ta.tp);
				break;
			case DELETE_TASK:
				PRINTF(2,"DELETE_TASK %s\n",ta.tp->task->getName().c_str());
				deleteTask(ta.tp);
				break;
			case CONDITIONALLY_IDLE:
				PRINTF(2,"CONDITIONALLY_IDLE\n");
				// TODO: this does not work. If a background task has been
				// created, we need to idle, but gtpid has changed
				if ((mainThread==NULL) && runScheduler) {
					// no new task has been created yet, and only this function
					// can trigger new task creation
					launchIdleTask();
				}
				break;
			case WAIT_CANCELLED:
				PRINTF(2,"WAIT_CANCELLED\n");
				break;
		}
		unlockScheduler();
		PRINTF(3,"rsl:Unlocked\n");
	}
	return 0;
}

int TaskScheduler::startScheduler() 
{
	assert(!runScheduler);
	runScheduler = true;
	enqueueAction(START_IDLE_TASK,boost::shared_ptr<ThreadParameters>());
	int res = pthread_create(&aqid,NULL,scheduler_thread,this);
	return res;
}

int TaskScheduler::stopScheduler()
{
	PRINTF(2,"Stopping scheduler (%d)\n",runScheduler);
	if (!runScheduler) return 0;
	runScheduler = false;
	enqueueAction(WAIT_CANCELLED,boost::shared_ptr<ThreadParameters>());
	pthread_join(aqid,NULL);
	aqid = 0;
	PRINTF(2,"Cleaning-up action queue (%d)\n",actionQueue.size());

	pthread_mutex_lock(&aqMutex);
	actionQueue.clear();
	pthread_mutex_unlock(&aqMutex);
	PRINTF(2,"Scheduler stopped\n");
	return 0;
}
	

void TaskScheduler::generateTaskList(std::vector<task_manager_msgs::TaskDescription> & tlist) const
{
	TaskDirectory::const_iterator it;
	unsigned int i = 0;
	for (it=tasks.begin();it!=tasks.end();it++,i++) {
        tlist.push_back(it->second->getDescription());
	}
}


void TaskScheduler::generateTaskStatus(std::vector<task_manager_msgs::TaskStatus> & running,
                std::vector<task_manager_msgs::TaskStatus> & zombies) 
{
	TaskSet::const_iterator it;
	lockScheduler();
	for (it=runningThreads.begin();it!=runningThreads.end();it++) 
	{
        running.push_back(it->second->getRosStatus());
	}
	for (it=zombieThreads.begin();it!=zombieThreads.end();it++) 
	{
        zombies.push_back(it->second->getRosStatus());
	}
	// tp.printToFile(stdout);
	unlockScheduler();
}

void TaskScheduler::generateTaskListLight(std::vector<task_manager_msgs::TaskDescription> &input,std::vector<task_manager_msgs::TaskDescriptionLight> &output) const
{
	std::vector<task_manager_msgs::TaskDescription> tasklist=input; 
	for (unsigned int i = 0;i<tasklist.size();i++) 
	{
        task_manager_msgs::TaskDescriptionLight current_task;
        
        current_task.name=tasklist[i].name;
        current_task.description=tasklist[i].description;
        current_task.periodic=tasklist[i].periodic;
        current_task.timeout_s=tasklist[i].timeout_s;
        for (unsigned int j = 0;j<tasklist[i].config.parameters.size();j++) 
        {
        	if ( (tasklist[i].config.parameters[j].name!= "task_rename") && (tasklist[i].config.parameters[j].name!= "main_task") && (tasklist[i].config.parameters[j].name!= "task_period") && (tasklist[i].config.parameters[j].name!= "task_timeout"))
        	{
        		task_manager_msgs::TaskParameter current_parameter;
        		current_parameter.name=tasklist[i].config.parameters[j].name;
        		current_parameter.description=tasklist[i].config.parameters[j].description;
        		current_parameter.type=tasklist[i].config.parameters[j].type;
        		 
        		std::ostringstream ostr;
        		if (current_parameter.type=="double")
        		{
        			unsigned int k=0;
        			while(tasklist[i].config.max.doubles[k].name != current_parameter.name)
        			{
    					k++;
        			}
        			
        			//max
        			ostr.str("");
        			ostr << tasklist[i].config.max.doubles[k].value;
        			current_parameter.max=ostr.str();
        			ostr.str("");
					
					//min
					ostr << tasklist[i].config.min.doubles[k].value;
        			current_parameter.min=ostr.str();
					ostr.str("");
					
					//default
					ostr << tasklist[i].config.dflt.doubles[k].value;
					current_parameter.dflt=ostr.str();
					ostr.str("");
        		
        		}
        		else if (current_parameter.type=="bool")
        		{
        			unsigned int k=0;
        			while(tasklist[i].config.max.bools[k].name!=current_parameter.name)
        			{
    					k++;
        			}
        			
        			//max
					if (tasklist[i].config.max.bools[k].value==1)
					{
						current_parameter.max="True";
        				
					}
					else if (tasklist[i].config.max.bools[k].value==0)
					{	
						current_parameter.max="False";
					}
        			
					//min
					if (tasklist[i].config.min.bools[k].value==1)
					{
						current_parameter.min="True";
					}
					else if (tasklist[i].config.min.bools[k].value==0)
					{	
						current_parameter.min="False";
					}
					
					//default
					if (tasklist[i].config.dflt.bools[k].value==1)
					{
						current_parameter.dflt="True";
					}
					else if (tasklist[i].config.dflt.bools[k].value==0)
					{	
						current_parameter.dflt="False";
					}
        		
        		}
        		else if(current_parameter.type=="int")
        		{
        			unsigned int k=0;
        			while(tasklist[i].config.max.ints[k].name!=current_parameter.name)
        			{
    					k++;
        			}
        			//max
        			ostr.str("");
        			ostr << tasklist[i].config.max.ints[k].value;
					current_parameter.max=ostr.str();
					ostr.str("");
					
					//min
					ostr << tasklist[i].config.min.ints[k].value;
					current_parameter.min=ostr.str();
					ostr.str("");
					
					//default
					ostr <<tasklist[i].config.dflt.ints[k].value;
					current_parameter.dflt=ostr.str();
					ostr.str("");
        		} 
        		else if (current_parameter.type=="str")
        		{
        			unsigned int k=0;
        			while(tasklist[i].config.max.ints[k].name!=current_parameter.name)
        			{
    					k++;
        			}
        			
        			
        			//max
					current_parameter.max=tasklist[i].config.max.strs[k].value;
					//min
					current_parameter.min=tasklist[i].config.min.strs[k].value;
					//default
					current_parameter.dflt=tasklist[i].config.dflt.strs[k].value;
        		}
        		else
        		{
        			cout<<"ERROR TYPE NOT LEGAL\n";
        		}
        		current_task.parameters.push_back(current_parameter);
	       	}
	       	
        }
        try
        {
        	output.push_back(current_task);
        }
        catch(...)
        {
        	cout<<"Error\n";
        }
        
        
	}

}


