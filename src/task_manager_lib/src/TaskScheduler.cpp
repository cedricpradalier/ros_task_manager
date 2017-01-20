#include <dirent.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/common.h>

#include "task_manager_lib/SequenceTask.h"
#include "task_manager_lib/DynamicTask.h"
#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/TaskHistory.h"
#include <dynamic_reconfigure/config_tools.h>

using namespace std;
using namespace task_manager_lib;



unsigned int TaskScheduler::ThreadParameters::gtpid = 0;
unsigned int TaskScheduler::debug = 1;
const double TaskScheduler::DELETE_TIMEOUT=2.0;
const double TaskScheduler::IDLE_TIMEOUT=0.5;
const unsigned int TaskScheduler::history_size=10;

TaskScheduler::ThreadParameters::ThreadParameters(ros::Publisher pub, TaskScheduler *ts, 
        boost::shared_ptr<TaskDefinitionBase> td, double tperiod) : statusPub(pub)
{
    gtpid += 1;
    tpid = gtpid;
    task = td->instantiate();
    that = ts;
    period = tperiod;
    foreground = true;
}


TaskScheduler::ThreadParameters::ThreadParameters(
        const TaskScheduler::ThreadParameters &tp)
{
    ROS_ERROR("ThreadParameters copy constructor called");
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
}

TaskScheduler::ThreadParameters::~ThreadParameters()
{
    if (TaskScheduler::debug >= 3) {
        if (!task_mutex.try_lock()) {
            ROS_ERROR("Task '%s': task_mutex locked on tp destructor",task->getName().c_str());
        } else {
            ROS_INFO("Task '%s': task_mutex unlocked on tp destructor",task->getName().c_str());
        }
        task_mutex.unlock();

        if (!aperiodic_task_mutex.try_lock()) {
            ROS_ERROR("Task '%s': aperiodic_task_mutex locked on tp destructor",task->getName().c_str());
        } else {
            ROS_INFO("Task '%s': aperiodic_task_mutex unlocked on tp destructor",task->getName().c_str());
        }
        aperiodic_task_mutex.unlock();
    }
}

TaskScheduler::TaskScheduler(ros::NodeHandle & nh, boost::shared_ptr<TaskDefinitionBase> tidle, double deftPeriod)
{
    runScheduler = false;

    idle = tidle;
    idleTimeout = 0.001;

    tasks["idle"] = idle;
    defaultPeriod = deftPeriod;
    startingTime = ros::Time::now().toSec();

    mainThread.reset();

    ROS_INFO("Task scheduler created: debug %d",debug);

    n = nh;
    startTaskSrv = nh.advertiseService("start_task", &TaskScheduler::startTask,this);
    stopTaskSrv = nh.advertiseService("stop_task", &TaskScheduler::stopTask,this);
    getTaskListSrv = nh.advertiseService("get_all_tasks", &TaskScheduler::getTaskList,this);
    getTaskListLightSrv =nh.advertiseService("get_all_tasks_light", &TaskScheduler::getTaskListLight,this);
    getAllTaskStatusSrv = nh.advertiseService("get_all_status", &TaskScheduler::getAllTaskStatus,this);
    getHistorySrv = nh.advertiseService("get_history", &TaskScheduler::getHistory,this);
    executeSequenceTasksSrv=nh.advertiseService("execute_sequence", &TaskScheduler::executeTaskSequence ,this);
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

    if (TaskScheduler::debug >= 3) {
        if (!aqMutex.try_lock()) {
            printf("aqMutex locked on sched destructor\n");
        } else {
            printf("aqMutex unlocked on sched destructor\n");
        }
        aqMutex.unlock();

        if (!scheduler_mutex.try_lock()) {
            printf("Scheduler_mutex locked on sched destructor\n");
        } else {
            printf("Scheduler_mutex unlocked on sched destructor\n");
        }
        scheduler_mutex.unlock();
    }
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
    lastKeepAlive = ros::Time::now();
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

bool TaskScheduler::executeTaskSequence(task_manager_lib::ExeTaskSequence::Request  &req,task_manager_lib::ExeTaskSequence::Response &res)
{
    lastKeepAlive = ros::Time::now();
    launchTaskSequence(req.sequence,res.id);
    return true;
}

bool TaskScheduler::getHistory(task_manager_lib::GetHistory::Request  &req, task_manager_lib::GetHistory::Response &res)
{
    lastKeepAlive = ros::Time::now();
    generateHistory(res.history);
    return true;
}


int TaskScheduler::terminateAllTasks()
{
    TaskSet copy = runningThreads;
    TaskSet::iterator it;
    PRINTF(1,"Terminating all tasks");
    for (it = copy.begin();it!=copy.end();it++) {
        // delete pointer and empty the list of running tasks
        terminateTask(it->second);
    }
    runningThreads.clear();
    mainThread.reset();
    return 0;
}

void TaskScheduler::addTask(boost::shared_ptr<TaskDefinitionBase> td) 
{
    PRINTF(1,"Adding task %s",td->getName().c_str());
    TaskDirectory::const_iterator tit = tasks.find(td->getName());
    if (tit != tasks.end()) {
        ROS_WARN("Warning: overwriting task '%s'",td->getName().c_str());
    }
    tasks.insert(std::pair< std::string,boost::shared_ptr<TaskDefinitionBase> >(td->getName(),td));
}

void TaskScheduler::loadTask(const std::string & filename, boost::shared_ptr<TaskEnvironment> env)
{
    boost::shared_ptr<TaskDefinitionBase> td(new DynamicTask(filename, env));

    addTask(td);
}

void TaskScheduler::configureTasks()
{
    TaskDirectory::const_iterator tit;
    unsigned int taskId = 0;
    for (tit = tasks.begin();tit!=tasks.end();tit++,taskId++) {
        // Try loading from the parameter server / launch file
        TaskParameters tp = tit->second->getParametersFromServer(n);
        // printf("TS %s param from server\n",tit->second->getName().c_str());
        // tp.print();
        std::string rename;
        if (tp.getParameter("task_rename",rename) && !rename.empty()) {
            tit->second->setName(rename);
        }
        tit->second->doConfigure(taskId, tp);
    }
}

#define DLL_EXT "so"

static int dllfilter(const struct dirent * d) {
    std::string name(d->d_name);
    size_t pos = name.find_last_of(".");
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
	    PRINTF(2,"Scandir: %s / %s",dirname.c_str(),namelist[n]->d_name);
            std::string fname = dirname + "/" + namelist[n]->d_name;
            loadTask(fname,env);
            free(namelist[n]);
        }
        free(namelist);
    }
}

void TaskScheduler::clearAllDynamicTasks() {
    TaskDirectory::iterator it;
    std::vector<TaskDirectory::iterator> tbd;

    if (runScheduler) {
        terminateAllTasks();
        stopScheduler();
    }

    for (it=tasks.begin();it!=tasks.end();it++) {
        boost::shared_ptr<DynamicTask> dt = boost::dynamic_pointer_cast<DynamicTask>(it->second);
        if (dt) {
            tbd.push_back(it);
        }
    }
    for (unsigned int i=0;i<tbd.size();i++) {
        tasks.erase(tbd[i]);
    }
}


TaskScheduler::TaskId TaskScheduler::launchIdleTask()
{
    double period = defaultPeriod;

    if (mainThread) {
        if (mainThread->task->isAnInstanceOf(idle)) {
            return mainThread->tpid;
        } else {
            terminateTask(mainThread);
        }
    }

    // Finally create the thread responsible for running the task
    PRINTF(3,"lit:Locking");
    {
        boost::unique_lock<boost::mutex> lock(scheduler_mutex);
        PRINTF(3,"lit:Locked");
        mainThread = boost::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, idle, period));
        mainThread->foreground = true;
        runningThreads[mainThread->tpid] = mainThread;
        if (debug>=3) printTaskSet("After launch idle",runningThreads);
    }
    PRINTF(3,"lit:Unlocked");

    mainThread->tid = boost::shared_ptr<boost::thread>(new boost::thread(&TaskScheduler::runTask,this,mainThread));

    return mainThread->tpid;
}

TaskScheduler::TaskId TaskScheduler::launchTask(boost::shared_ptr<ThreadParameters> tp)
{

    if (tp->foreground) {
        terminateTask(mainThread);
    }

    PRINTF(3,"lt:Locking");
    {
        boost::unique_lock<boost::mutex> lock(scheduler_mutex);
        PRINTF(3,"lt:Locked");

        if (tp->foreground) {
            mainThread = tp;
        }
        runningThreads[tp->tpid] = tp;
        if (debug>=3) printTaskSet("After launch",runningThreads);
    }
    PRINTF(3,"lt:Unlocked");

    boost::unique_lock<boost::mutex> lock(tp->task_mutex);
    try {
        tp->tid = boost::shared_ptr<boost::thread>(new boost::thread(&TaskScheduler::runTask,this,tp));
        if (tp->foreground) {
            removeConditionalIdle();
        }
    } catch (const boost::thread_resource_error & e) {
        // there is a risk to check here
        tp->task_condition.notify_all();
        if (tp->foreground) {
            launchIdleTask();
        }
        enqueueAction(ros::Time::now()+ros::Duration(DELETE_TIMEOUT),DELETE_TASK,tp);
        return 0;
    }
    return tp->tpid;
}

TaskScheduler::TaskId TaskScheduler::launchTask(const std::string & taskname, 
        const TaskParameters & tp)
{
    bool foreground = true;
    double period = defaultPeriod;
    TaskDirectory::const_iterator tdit;
    tdit = tasks.find(taskname);
    if (tdit==tasks.end()) {
        ROS_ERROR("Impossible to find task '%s'",taskname.c_str());
        return -1;
    }
    if (tdit->second->getStatus() != TaskStatus::TASK_CONFIGURED) {
        ROS_ERROR("Refusing to run task '%s' because it is not CONFIGURED",taskname.c_str());
        return -1;
    }

    // See if some runtime period has been defined in the parameters
    if (!tp.getParameter("task_period",period)) {
        ROS_ERROR("Missing required parameter task_period");
        return -1;
    }
    tp.getParameter("foreground",foreground); // ignore return

    // Finally create the thread responsible for running the task
    boost::shared_ptr<ThreadParameters> tparam =
        boost::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, tdit->second, period));
    tparam->params = tp;
    tparam->foreground = foreground;
    tparam->running = false;

    PRINTF(3,"lt:Locking");
    {
        boost::unique_lock<boost::mutex> lock(tparam->task_mutex);
        PRINTF(3,"lt:Locked");

        enqueueAction(START_TASK,tparam);

        PRINTF(3,"lt:Wait condition");
        tparam->task_condition.wait(lock);
        PRINTF(3,"lt:Locked");
    }
    PRINTF(3,"lt:Unlocked");

    return tparam->tpid;
}

void TaskScheduler::runAperiodicTask(boost::shared_ptr<ThreadParameters> tp)
{
    // Just a barrier...
    tp->aperiodic_task_mutex.lock();
    tp->aperiodic_task_mutex.unlock();

    tp->task->doIterate();
    // WARNING: this might be misinterpreted. Check this.
    tp->aperiodic_task_condition.notify_all();
}

void TaskScheduler::runTask(boost::shared_ptr<ThreadParameters> tp)
{
    try {
        double tstart = ros::Time::now().toSec();
        PRINTF(3,"lt:Signaling and unlocking");
        {
            boost::unique_lock<boost::mutex> lock(tp->task_mutex);
            tp->task_condition.notify_all();
        }
        PRINTF(3,"lt:Unlocked");
        tp->updateStatus(ros::Time::now());

        tp->running = true;
        try {
            tp->task->doInitialise(tp->tpid,tp->params);
            tp->updateStatus(ros::Time::now());
            if (tp->status != TaskStatus::TASK_INITIALISED) {
                cleanupTask(tp);
                return;
            }
        } catch (const std::exception & e) {
            tp->task->debug("Exception %s",e.what());
            tp->updateStatus(ros::Time::now());
            cleanupTask(tp);
            return ;
        }
        tp->running = true;
        ROS_INFO("Running task '%s' at period %f main %d timeout %f",tp->task->getName().c_str(),tp->period,(tp==mainThread),tp->task->getTimeout());

        if (tp->task->isPeriodic()) {
            ros::Rate rate(1. / tp->period);
            PRINTF(2,"Initialisation done");
            while (1) {
                double t0 = ros::Time::now().toSec();
                if (mainThread && (!mainThread->isAnInstanceOf(idle)) && (t0 - lastKeepAlive.toSec() > 1.0)) {
                    tp->task->debug("KEEPALIVE failed");
                    tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",ros::Time(t0));
                    break;
                }

                if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
                    tp->task->debug("TIMEOUT");
                    tp->setStatus(task_manager_msgs::TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",ros::Time(t0));
                    break;
                }

                try {
                    // tp->task->debug("Iterating...");
                    tp->task->doIterate();
                    tp->updateStatus(now());
                } catch (const std::exception & e) {
                    tp->task->debug("Exception %s",e.what());
                    tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "Interrupted by Exception",ros::Time(t0));
                    tp->updateStatus(now());
                    cleanupTask(tp);
                    return ;
                }
                if (tp->status != task_manager_msgs::TaskStatus::TASK_RUNNING) {
                    ROS_INFO("Task '%s' not running anymore (%d)",tp->task->getName().c_str(),tp->status);
                    break;
                }
                boost::this_thread::interruption_point();
                rate.sleep();
            }
        } else {
            bool first = true;
            boost::unique_lock<boost::mutex> lock(tp->aperiodic_task_mutex);
            boost::thread id(&TaskScheduler::runAperiodicTask,this,tp);
            while (1) {
                double t0 = ros::Time::now().toSec();
                if (mainThread && (!mainThread->isAnInstanceOf(idle)) && (t0 - lastKeepAlive.toSec() > 1.0)) {
                    tp->task->debug("KEEPALIVE failed");
                    tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",ros::Time(t0));
                    break;
                }
                if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
                    tp->task->debug("TIMEOUT");
                    tp->setStatus(task_manager_msgs::TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",ros::Time(t0));
                    break;
                }
                tp->updateStatus(ros::Time::now());
                if (!first && tp->status != task_manager_msgs::TaskStatus::TASK_RUNNING) {
                    ROS_INFO("Task '%s' not running anymore",tp->task->getName().c_str());
                    break;
                }

                double t1 = ros::Time::now().toSec();
                double ttimeout = std::max(1e-3,(tp->period - (t1-t0)));
                boost::posix_time::milliseconds dtimeout(ttimeout*1000);
                tp->aperiodic_task_condition.timed_wait(lock,dtimeout);
                first = false;
            }
            if (tp->status != task_manager_msgs::TaskStatus::TASK_COMPLETED) {
                id.interrupt();
            }
            id.join();
        }
        // tp->task->debug("Out of the loop");
    } catch (const boost::thread_interrupted & e) {
        // Ignore, we just want to make sure we get to the next line
        tp->setStatus(task_manager_msgs::TaskStatus::TASK_INTERRUPTED, "Interrupted by Exception",ros::Time::now());
    }
    cleanupTask(tp);
}

void TaskScheduler::terminateTask(boost::shared_ptr<ThreadParameters> tp)
{
    if (!tp) return;
    PRINTF(1,"Terminating thread %s",tp->task->getName().c_str());
    tp->tid->interrupt();
    tp->tid->join();
    tp->running = false;

    PRINTF(2,"Thread cancelled");
    return;
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

void TaskScheduler::cleanupTask(boost::shared_ptr<ThreadParameters> tp)
{
    if (tp == NULL) return ;
    PRINTF(1,"Cleaning up task %d:%s",tp->tpid,tp->task->getName().c_str());
    tp->task->doTerminate();
    ROS_INFO("Task '%s' terminated",tp->task->getName().c_str());
    tp->status |= task_manager_msgs::TaskStatus::TASK_TERMINATED;
    tp->statusTime = now();
    tp->statusString = "terminated";
    tp->updateStatus(now());

    boost::unique_lock<boost::mutex> lock(scheduler_mutex);
    zombieThreads.insert(TaskSetItem(tp->tpid,tp));
    TaskSet::iterator tsit = runningThreads.find(tp->tpid);
    assert(tsit != runningThreads.end());
    runningThreads.erase(tsit);

    if (debug>=3) printTaskSet("Zombies at cleanup",zombieThreads);
    if (debug>=3) printTaskSet("Running at cleanup",runningThreads);


    if (tp->foreground) {
        mainThread.reset();
        if (!tp->task->isAnInstanceOf(idle)) {
            enqueueAction(ros::Time::now()+ros::Duration(IDLE_TIMEOUT),CONDITIONALLY_IDLE,tp);
        }
    }
    enqueueAction(ros::Time::now()+ros::Duration(DELETE_TIMEOUT),DELETE_TASK,tp);

    // There is a risk that we trigger the execution of a new task before
    // starting idle here. In addition, we need one condition per task
    tp->task_condition.notify_all();
}

void TaskScheduler::deleteTask(boost::shared_ptr<ThreadParameters> tp)
{
    TaskSet::iterator it = zombieThreads.find(tp->tpid);
    assert(it != zombieThreads.end());
    zombieThreads.erase(it);
    if (tp->tid) {
        tp->tid->join();
    }
    tp.reset();
}

int TaskScheduler::waitTaskCompletion(TaskId id, double timeout)
{
    TaskSet::iterator it;
    boost::unique_lock<boost::mutex> lock(scheduler_mutex);
    if (debug>=3) printTaskSet("Zombies when waiting",zombieThreads);
    if (debug>=3) printTaskSet("Running when waiting",runningThreads);

    it = zombieThreads.find(id);
    if (it != zombieThreads.end()) {
        return 0;
    }
    it = runningThreads.find(id);
    if (it == runningThreads.end()) {
        ROS_ERROR("Cannot find reference to task %d",id);
        return -1;
    }
    boost::posix_time::milliseconds dtimeout(timeout*1000);
    it->second->task_condition.timed_wait(lock,dtimeout);
    return 0;
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

void TaskScheduler::removeConditionalIdle()
{
    PRINTF(3,"rci:Locking");
    boost::unique_lock<boost::mutex> lock(aqMutex);
    PRINTF(3,"rci:Locked");

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

    PRINTF(3,"rci:Signalling");
    aqCond.notify_all();
    PRINTF(3,"rci:Unlocked");
}

TaskScheduler::ThreadAction TaskScheduler::getNextAction()
{
    // Warning: add cancel management
    ros::Time t;
    ThreadAction ta;
    ActionQueue::iterator it;
    PRINTF(3,"gna:Locking");
    boost::unique_lock<boost::mutex> lock(aqMutex);
    PRINTF(3,"gna:Locked");
    try {
        // TODO: this must get the next action in time
        if (actionQueue.empty()) {
            PRINTF(3,"gna:Cond Wait");
            aqCond.wait(lock);
            PRINTF(3,"gna:Locked");
        }
        while (1) {
            t = ros::Time::now();
            it = actionQueue.begin();
            assert(it != actionQueue.end());
            if (runScheduler) {
                if (it->first <= t.toSec()) {
                    ta = it->second;
                    actionQueue.erase(it);
                    PRINTF(2,"Dequeueing action %.3f %s -- %s",it->first,actionString(ta.type),
                            ta.tp?(ta.tp->task->getName().c_str()):"none");
                } else {
                    // wait for the right time or another action to be inserted
                    PRINTF(3,"gna:Cond TWait");
                    boost::posix_time::milliseconds dtimeout((it->first-t.toSec())*1000);
                    aqCond.timed_wait(lock,dtimeout);
                    PRINTF(3,"gna:Locked");
                    continue;
                }
            } else {
                ta = it->second;
                actionQueue.erase(it);
                PRINTF(2,"Dequeueing action %.3f %s -- %s",it->first,actionString(ta.type),
                        ta.tp?(ta.tp->task->getName().c_str()):"none");
            }
            break;
        }
    } catch (const boost::thread_interrupted & e) {
        ta.type = WAIT_CANCELLED;
        ta.tp.reset();
    }
    PRINTF(3,"gna:Unlocking");
    return ta;
}

void TaskScheduler::enqueueAction(ActionType type,boost::shared_ptr<ThreadParameters> tp)
{
    ThreadAction ta;
    PRINTF(3,"ea:Locking");
    boost::unique_lock<boost::mutex> lock(aqMutex);
    PRINTF(3,"ea:Locked");
    // if (!runScheduler) return;
    double when = ros::Time::now().toSec();
    PRINTF(2,"Enqueueing action %.3f %s -- %s",when,actionString(type),
            tp?(tp->task->getName().c_str()):"none");

    ta.type = type;
    ta.tp = tp;
    actionQueue[when] = ta;
    PRINTF(3,"ea:Signalling");
    aqCond.notify_all();
    PRINTF(3,"ea:Unlocking");
}

void TaskScheduler::enqueueAction(const ros::Time & when,  ActionType type,boost::shared_ptr<ThreadParameters> tp)
{
    ThreadAction ta;
    PRINTF(3,"ea:Locking");
    boost::unique_lock<boost::mutex> lock(aqMutex);
    PRINTF(3,"ea:Locked");
    // if (!runScheduler) return;
    PRINTF(2,"Enqueing action %.3f %s -- %s",when.toSec(),actionString(type),
            tp?(tp->task->getName().c_str()):"none");

    ta.type = type;
    ta.tp = tp;
    actionQueue[when.toSec()] = ta;
    PRINTF(3,"ea:Signalling");
    aqCond.notify_all();
    PRINTF(3,"ea:Unlocking");
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
        case NO_ACTION:
            return "NO_ACTION";
            break;
        default:
            break;
    }
    return "Unknown";
}

int TaskScheduler::runSchedulerLoop()
{
    // Default:
    // pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    while (1) {
        PRINTF(2,"Waiting next action (%d)",(int)(actionQueue.size()));
        if (!runScheduler && actionQueue.empty()) {
            break;
        }

        ThreadAction ta = getNextAction();
        PRINTF(2,"%.3f: got action %s %s",ros::Time::now().toSec(),actionString(ta.type),
                ta.tp?(ta.tp->task->getName().c_str()):"none");
        PRINTF(3,"rsl:Locking");
        switch (ta.type) {
            case START_IDLE_TASK:
                PRINTF(2,"START_IDLE_TASK");
                launchIdleTask();
                break;
            case START_TASK:
                PRINTF(2,"START_TASK %s",ta.tp->task->getName().c_str());
                launchTask(ta.tp);
                break;
            case DELETE_TASK:
                PRINTF(2,"DELETE_TASK %s",ta.tp->task->getName().c_str());
                deleteTask(ta.tp);
                break;
            case CONDITIONALLY_IDLE:
                PRINTF(2,"CONDITIONALLY_IDLE");
                if ((mainThread==NULL) && runScheduler) {
                    // no new task has been created yet, and only this function
                    // can trigger new task creation
                    launchIdleTask();
                }
                break;
            case WAIT_CANCELLED:
                PRINTF(2,"WAIT_CANCELLED");
                break;
            case NO_ACTION:
                PRINTF(2,"NO_ACTION");
                break;
        }
        PRINTF(3,"rsl:Unlocked");
    }
    return 0;
}

int TaskScheduler::startScheduler() 
{
    assert(!runScheduler);
    runScheduler = true;
    enqueueAction(START_IDLE_TASK,boost::shared_ptr<ThreadParameters>());
    aqid = boost::thread(&TaskScheduler::runSchedulerLoop,this);
    return 0;
}

int TaskScheduler::stopScheduler()
{
    PRINTF(2,"Stopping scheduler (%d)",runScheduler);
    if (!runScheduler) return 0;
    runScheduler = false;
    enqueueAction(WAIT_CANCELLED,boost::shared_ptr<ThreadParameters>());
    aqid.join();
    PRINTF(2,"Cleaning-up action queue (%d)",(int)(actionQueue.size()));

    boost::unique_lock<boost::mutex> lock(aqMutex);
    actionQueue.clear();
    PRINTF(2,"Scheduler stopped");
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
    boost::unique_lock<boost::mutex> lock(scheduler_mutex);
    for (it=runningThreads.begin();it!=runningThreads.end();it++) 
    {
        running.push_back(it->second->getRosStatus());
    }
    for (it=zombieThreads.begin();it!=zombieThreads.end();it++) 
    {
        zombies.push_back(it->second->getRosStatus());
    }
    // tp.printToFile(stdout);
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

#if ROS_VERSION_MINIMUM(1, 8, 0)
// #pragma message("Compiling for ROS Fuerte")
        for (unsigned int g=0;g<tasklist[i].config.groups.size();g++) {
            for (unsigned int j = 0;j<tasklist[i].config.groups[g].parameters.size();j++) {
                if ( (tasklist[i].config.groups[g].parameters[j].name!= "task_rename") 
                        && (tasklist[i].config.groups[g].parameters[j].name!= "foreground") 
                        && (tasklist[i].config.groups[g].parameters[j].name!= "task_period") 
                        && (tasklist[i].config.groups[g].parameters[j].name!= "task_timeout"))
                {
                    task_manager_msgs::TaskParameter current_parameter;
                    current_parameter.name=tasklist[i].config.groups[g].parameters[j].name;
                    current_parameter.description=tasklist[i].config.groups[g].parameters[j].description;
                    current_parameter.type=tasklist[i].config.groups[g].parameters[j].type;
#else
#pragma message("Compiling for ROS Electric Turtle")
                    for (unsigned int j = 0;j<tasklist[i].config.parameters.size();j++) {
                        if ( (tasklist[i].config.parameters[j].name!= "task_rename") 
                                && (tasklist[i].config.parameters[j].name!= "foreground") 
                                && (tasklist[i].config.parameters[j].name!= "task_period") 
                                && (tasklist[i].config.parameters[j].name!= "task_timeout"))
                        {
                            task_manager_msgs::TaskParameter current_parameter;
                            current_parameter.name=tasklist[i].config.parameters[j].name;
                            current_parameter.description=tasklist[i].config.parameters[j].description;
                            current_parameter.type=tasklist[i].config.parameters[j].type;
#endif



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
                                while(tasklist[i].config.max.strs[k].name!=current_parameter.name)
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
                                PRINTF(1,"TYPE NOT LEGAL");
                            }
                            current_task.parameters.push_back(current_parameter);
                        }


#if ROS_VERSION_MINIMUM(1, 8, 0)

                    }
                }
#else
            }
#endif


            output.push_back(current_task);



        }

    }

    void TaskScheduler::generateHistory(std::vector<task_manager_msgs::TaskHistory> &output) 
    {
        std::ostringstream ostr;
        for (unsigned i=0;i<history.size();i++)
        {
            task_manager_msgs::TaskHistory task;
            task.tid=history[i].getid();
            task.name=history[i].getname();
            task.start_time=history[i].getstartTime();
            task.end_time=history[i].getendTime();
            task.status=history[i].getstatus();

            //bool
            for (unsigned int j = 0;j<history[i].getparams().bools.size();j++) 
            {
                if (history[i].getparams().bools[j].name!="foreground" && history[i].getparams().bools[j].name!="task_rename" && history[i].getparams().bools[j].name!="task_period" && history[i].getparams().bools[j].name!="task_period")
                {
                    task_manager_msgs::TaskParameter current_parameter;
                    current_parameter.name=history[i].getparams().bools[j].name;
                    if (history[i].getparams().bools[j].value==1)
                    {
                        current_parameter.value="True";
                    }
                    else
                    {
                        current_parameter.value="False";
                    }
                    current_parameter.type="bool";
                    task.parameters.push_back(current_parameter);
                }
            }

            //int
            for (unsigned int j = 0;j<history[i].getparams().ints.size();j++) 
            {
                if (history[i].getparams().ints[j].name!="foreground" && history[i].getparams().ints[j].name!="task_rename" && history[i].getparams().ints[j].name!="task_period" && history[i].getparams().ints[j].name!="task_period")
                {

                    task_manager_msgs::TaskParameter current_parameter;
                    current_parameter.name=history[i].getparams().ints[j].name;
                    ostr.str("");
                    ostr <<history[i].getparams().ints[j].value;
                    current_parameter.value=ostr.str();
                    ostr.str("");
                    task.parameters.push_back(current_parameter);

                }
            }

            //strs
            for (unsigned int j = 0;j<history[i].getparams().strs.size();j++) 
            {
                if (history[i].getparams().strs[j].name!="foreground" && history[i].getparams().strs[j].name!="task_rename" && history[i].getparams().strs[j].name!="task_period" && history[i].getparams().strs[j].name!="task_period")
                {
                    task_manager_msgs::TaskParameter current_parameter;
                    current_parameter.name=history[i].getparams().strs[j].name;
                    current_parameter.value=history[i].getparams().strs[j].value;
                    current_parameter.type="str";
                    task.parameters.push_back(current_parameter);
                }
            }

            //double
            for (unsigned int j = 0;j<history[i].getparams().doubles.size();j++) 
            {
                if (history[i].getparams().doubles[j].name!="foreground" && history[i].getparams().doubles[j].name!="task_rename" && history[i].getparams().doubles[j].name!="task_period" && history[i].getparams().doubles[j].name!="task_period")
                {

                    task_manager_msgs::TaskParameter current_parameter;
                    current_parameter.name=history[i].getparams().doubles[j].name;
                    ostr.str("");
                    ostr <<history[i].getparams().doubles[j].value;
                    current_parameter.value=ostr.str();
                    current_parameter.type="double";
                    task.parameters.push_back(current_parameter);
                }
            }

            output.push_back(task);
        }

    }

    void TaskScheduler::launchTaskSequence(std::vector<task_manager_msgs::TaskDescriptionLight> &tasks, int &id) 
    {
        TaskDefinitionPtr st(new SequenceDef(tasks,this));
        boost::shared_ptr<ThreadParameters> tp(new ThreadParameters(statusPub, this, st, defaultPeriod ));
        tp->foreground = false;
        id=launchTask(tp);
    }

    int TaskScheduler::getstatus(unsigned int &taskid)
    {
        TaskSet::const_iterator it;
        for (it=runningThreads.begin();it!=runningThreads.end();it++) 
        {
            if (it->first==taskid)
            {
                return it->second->getRosStatus().status;
            }
        }
        for (it=zombieThreads.begin();it!=zombieThreads.end();it++) 
        {
            if (it->first==taskid)
            {
                return it->second->getRosStatus().status;
            }
        }
        return -1;
    }

    int TaskScheduler::terminateTask(unsigned int &taskid)
    {
        boost::unique_lock<boost::mutex> lock(scheduler_mutex);
        TaskSet::iterator it;
        for (it = runningThreads.begin();it!=runningThreads.end();it++) 
        {
            // delete pointer and empty the list of running tasks
            if (it->first==taskid)
            {
                lock.unlock();
                terminateTask(it->second);
                return 0;
            }
        }
        return 0;
    }

    void TaskScheduler::keepAliveSequence()
    {
        lastKeepAlive = ros::Time::now();
    }

    ros::NodeHandle TaskScheduler::getNodeHandle()
    {
        return n;
    }

