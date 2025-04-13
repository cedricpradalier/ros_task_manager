#include <dirent.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>


#include "task_manager_lib/TaskScheduler.h"
#include "task_manager_lib/DynamicTask.h"

using namespace std;
using namespace task_manager_lib;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
typedef task_manager_msgs::msg::TaskStatus TaskStatus;

unsigned int TaskScheduler::ThreadParameters::gtpid = 0;
unsigned int TaskScheduler::debug = 1;
const rclcpp::Duration TaskScheduler::DELETE_TIMEOUT(2,0);
const rclcpp::Duration TaskScheduler::IDLE_TIMEOUT(0,0.5*1e9);
const unsigned int TaskScheduler::history_size=10;

TaskScheduler::ThreadParameters::ThreadParameters(rclcpp::Publisher<task_manager_msgs::msg::TaskStatus>::SharedPtr pub, TaskScheduler *ts, 
        TaskDefinitionPtr td, double tperiod) : statusPub(pub)
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
    that = tp.that;
    RCLCPP_ERROR(that->node->get_logger(), "ThreadParameters copy constructor called");
    tpid = tp.tpid;
    tid = tp.tid;
    task = tp.task;
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
            RCLCPP_ERROR(that->node->get_logger(), "Task '%s': task_mutex locked on tp destructor",task->getName().c_str());
        } else {
            RCLCPP_INFO(that->node->get_logger(), "Task '%s': task_mutex unlocked on tp destructor",task->getName().c_str());
        }
        task_mutex.unlock();

        if (!aperiodic_task_mutex.try_lock()) {
            RCLCPP_ERROR(that->node->get_logger(), "Task '%s': aperiodic_task_mutex locked on tp destructor",task->getName().c_str());
        } else {
            RCLCPP_INFO(that->node->get_logger(), "Task '%s': aperiodic_task_mutex unlocked on tp destructor",task->getName().c_str());
        }
        aperiodic_task_mutex.unlock();
    }
}

TaskScheduler::TaskScheduler(std::shared_ptr<rclcpp::Node> node, TaskDefinitionPtr tidle, double deftPeriod) :
    node(node)
{
    runScheduler = false;

    idle = tidle;
    idleTimeout = 0.001;

    tasks["idle"] = idle;
    defaultPeriod = deftPeriod;
    startingTime = now().seconds();

    mainThread.reset();

    RCLCPP_INFO(node->get_logger(), "Task scheduler created: debug %d",debug);

    startTaskSrv = node->create_service<task_manager_msgs::srv::StartTask>("~/start_task", std::bind(&TaskScheduler::startTask,this,std::placeholders::_1,std::placeholders::_2));
    stopTaskSrv = node->create_service<task_manager_msgs::srv::StopTask>("~/stop_task", std::bind(&TaskScheduler::stopTask,this,std::placeholders::_1,std::placeholders::_2));
    getTaskListSrv = node->create_service<task_manager_msgs::srv::GetTaskList>("~/get_all_tasks", std::bind(&TaskScheduler::getTaskList,this,std::placeholders::_1,std::placeholders::_2));
    getAllTaskStatusSrv = node->create_service<task_manager_msgs::srv::GetAllTaskStatus>("~/get_all_status", std::bind(&TaskScheduler::getAllTaskStatus,this,std::placeholders::_1,std::placeholders::_2));
    setParamHandle = node->add_on_set_parameters_callback(std::bind(&TaskScheduler::reconfigure_callback, this, std::placeholders::_1));
#if 0
    getTaskListLightSrv =nh.advertiseService("get_all_tasks_light", &TaskScheduler::getTaskListLight,this);
    getHistorySrv = nh.advertiseService("get_history", &TaskScheduler::getHistory,this);
    executeSequenceTasksSrv=nh.advertiseService("execute_sequence", &TaskScheduler::executeTaskSequence ,this);
#endif
    statusPub = node->create_publisher<task_manager_msgs::msg::TaskStatus>("~/status",50);
    keepAliveSub = node->create_subscription<std_msgs::msg::Header>("~/keep_alive",1,std::bind(&TaskScheduler::keepAliveCallback,this,std::placeholders::_1));
    lastKeepAlive = now();
}

void TaskScheduler::keepAliveCallback(std_msgs::msg::Header::ConstSharedPtr) 
{
    lastKeepAlive = now();
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

bool TaskScheduler::startTask(const std::shared_ptr<task_manager_msgs::srv::StartTask::Request>  req,
        const std::shared_ptr<task_manager_msgs::srv::StartTask::Response> res )
{
    lastKeepAlive = now();
    if (debug>1) RCLCPP_INFO(node->get_logger(),"Start task request %d arguments",int(req->config.size()));
    TaskId id = launchTask(req->name,req->config);
    res->id = id;
    return true;
}

bool TaskScheduler::stopTask(const std::shared_ptr<task_manager_msgs::srv::StopTask::Request>  req,
        const std::shared_ptr<task_manager_msgs::srv::StopTask::Response> res )
{
    lastKeepAlive = now();
    if (req->id == -1) {
        TaskId id = launchIdleTask();
        res->id = id;
    } else {
        TaskSet::iterator it = runningThreads.find(req->id);
        if (it != runningThreads.end()) {
            terminateTask(it->second);
        }
        res->id = 0;
    }
    return true;
}

bool TaskScheduler::getTaskList(const std::shared_ptr<task_manager_msgs::srv::GetTaskList::Request>  ,
        const std::shared_ptr<task_manager_msgs::srv::GetTaskList::Response> res )
{
    lastKeepAlive = now();
    generateTaskList(res->tlist);
    return true;
}


bool TaskScheduler::getAllTaskStatus(const std::shared_ptr<task_manager_msgs::srv::GetAllTaskStatus::Request>  ,
        const std::shared_ptr<task_manager_msgs::srv::GetAllTaskStatus::Response> res )
{
    lastKeepAlive = now();
    generateTaskStatus(res->running_tasks,res->zombie_tasks);
    return true;
}

#if 0
bool TaskScheduler::getTaskListLight(task_manager_lib::GetTaskListLight::Request  &req, task_manager_lib::GetTaskListLight::Response &res )
{
    lastKeepAlive = now();
    task_manager_lib::GetTaskList::Response res1;
    generateTaskList(res1.tlist);
    generateTaskListLight(res1.tlist,res.tlist);
    return true;
}


bool TaskScheduler::executeTaskSequence(task_manager_lib::ExeTaskSequence::Request  &req,task_manager_lib::ExeTaskSequence::Response &res)
{
    lastKeepAlive = now();
    launchTaskSequence(req.sequence,res.id);
    return true;
}

bool TaskScheduler::getHistory(task_manager_lib::GetHistory::Request  &req, task_manager_lib::GetHistory::Response &res)
{
    lastKeepAlive = now();
    generateHistory(res.history);
    return true;
}
#endif


int TaskScheduler::terminateAllTasks()
{
    TaskSet copy = runningThreads;
    TaskSet::iterator it;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Terminating all tasks");
    for (it = copy.begin();it!=copy.end();it++) {
        // delete pointer and empty the list of running tasks
        terminateTask(it->second);
    }
    runningThreads.clear();
    mainThread.reset();
    return 0;
}

void TaskScheduler::addTask(TaskDefinitionPtr td) 
{
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Adding task %s",td->getName().c_str());
    TaskDirectory::const_iterator tit = tasks.find(td->getName());
    if (tit != tasks.end()) {
        RCLCPP_WARN(node->get_logger(), "Warning: overwriting task '%s'",td->getName().c_str());
    }
    td->setTaskId(tasks.size());
    tasks.insert(std::pair< std::string,TaskDefinitionPtr >(td->getName(),td));
}

void TaskScheduler::loadTask(const std::string & filename, TaskEnvironmentPtr env)
{
    try {
        RCLCPP_DEBUG(node->get_logger(), "Trying to load %s",filename.c_str());
        DynamicTask *dt = new DynamicTask(filename, env);
        TaskDefinitionPtr td(dt);

        if (dt->loadTask(false)) {
            addTask(td);
        }
    } catch (DynamicTask::DLLoadError & e) {
        RCLCPP_ERROR(node->get_logger(), "Ignoring file '%s': '%s'",filename.c_str(),e.what());
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
        TaskEnvironmentPtr env)
{
    struct dirent **namelist;
    int n;
    n = scandir(dirname.c_str(), &namelist, dllfilter, alphasort);
    if (n < 0)
        perror("scandir");
    else {
        while(n--) {
	    if (debug>1) RCLCPP_INFO(node->get_logger(), "Scandir: %s / %s",dirname.c_str(),namelist[n]->d_name);
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
        std::shared_ptr<DynamicTask> dt = std::dynamic_pointer_cast<DynamicTask,TaskDefinitionBase>(it->second);
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
    if (debug>1) RCLCPP_INFO(node->get_logger(), "lit:Locking");
    {
        std::unique_lock<std::mutex> lock(scheduler_mutex);
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lit:Locked");
        mainThread = std::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, idle, period));
        mainThread->foreground = true;
        runningThreads[mainThread->tpid] = mainThread;
        if (debug>=3) printTaskSet("After launch idle",runningThreads);
    }
    if (debug>1) RCLCPP_INFO(node->get_logger(), "lit:Unlocked");

    mainThread->tid = std::shared_ptr<boost::thread>(new boost::thread(&TaskScheduler::runTask,this,mainThread));

    return mainThread->tpid;
}

TaskScheduler::TaskId TaskScheduler::launchTask(std::shared_ptr<ThreadParameters> tp)
{

    if (tp->foreground) {
        if ((debug>1) && mainThread) RCLCPP_INFO(node->get_logger(), "lt:terminate mainThread %s",mainThread->task->getName().c_str());
        terminateTask(mainThread);
    }

    if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Locking (param)");
    {
        std::unique_lock<std::mutex> lock(scheduler_mutex);
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Locked");

        if (tp->foreground) {
            mainThread = tp;
        }
        runningThreads[tp->tpid] = tp;
        if (debug>=3) printTaskSet("After launch",runningThreads);
    }
    if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Unlocked (param)");

    std::unique_lock<std::mutex> lock(tp->task_mutex);
    try {
        tp->tid = std::shared_ptr<boost::thread>(new boost::thread(&TaskScheduler::runTask,this,tp));
        if (tp->foreground) {
            removeConditionalIdle();
        }
    } catch (const boost::thread_resource_error & e) {
        // there is a risk to check here
        tp->task_condition.notify_all();
        if (tp->foreground) {
            launchIdleTask();
        }
        enqueueAction(now()+DELETE_TIMEOUT,DELETE_TASK,tp);
        return 0;
    }
    return tp->tpid;
}

rcl_interfaces::msg::SetParametersResult TaskScheduler::reconfigure_callback(
        const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // for (const auto & parameter : parameters) {
    //     if (!some_condition) {
    //         result.successful = false;
    //         result.reason = "the reason it could not be allowed";
    //     }
    // }
    TaskSet::const_iterator it;
    std::unique_lock<std::mutex> lock(scheduler_mutex);
    for (it=runningThreads.begin();it!=runningThreads.end();it++) 
    {
        if (!it->second->task->isReadyForReconfigure()) {
            continue;
        }
        result = it->second->task->reconfigure_callback(parameters);
        if (!result.successful) {
            return result;
        }
    }
    return result;
}

TaskScheduler::TaskId TaskScheduler::launchTask(const std::string & taskname, 
        const TaskParameters & tp)
{
    bool foreground = true;
    double period = defaultPeriod;
    TaskDirectory::const_iterator tdit;
    tdit = tasks.find(taskname);
    if (tdit==tasks.end()) {
        RCLCPP_ERROR(node->get_logger(), "Impossible to find task '%s'",taskname.c_str());
        return -1;
    }
    // Get a config copy
    TaskConfig cfg = *(tdit->second->getConfig());
    if (debug>1) RCLCPP_INFO(node->get_logger(),"Loaded task paramter with %d arguments",int(tp.size()));
    cfg.loadConfig(tp,"");
    if (debug>1) cfg.printConfig();
    // See if some runtime period has been defined in the parameters
    period = cfg.get<double>("task_period");
    foreground = cfg.get<bool>("foreground");

    // Finally create the thread responsible for running the task
    std::shared_ptr<ThreadParameters> tparam =
        std::shared_ptr<ThreadParameters>(new ThreadParameters(statusPub, this, tdit->second, period));
    tparam->params = cfg;
    tparam->foreground = foreground;
    tparam->running = false;

    if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Locking (name)");
    {
        std::unique_lock<std::mutex> lock(tparam->task_mutex);
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Locked");

        enqueueAction(START_TASK,tparam);

        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Wait condition");
        tparam->task_condition.wait(lock);
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Locked");
    }
    if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Unlocked (name)");

    return tparam->tpid;
}

void TaskScheduler::runAperiodicTask(std::shared_ptr<ThreadParameters> tp)
{
    // Just a barrier...
    tp->aperiodic_task_mutex.lock();
    tp->aperiodic_task_mutex.unlock();

    // Forcing status to RUNNING to avoid task not appearing in task client
    tp->updateStatus(now());
    tp->task->doIterate();
    // WARNING: this might be misinterpreted. Check this.
    tp->aperiodic_task_condition.notify_all();
}

void TaskScheduler::runTask(std::shared_ptr<ThreadParameters> tp)
{
    try {
        double tstart = now().seconds();
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Signaling and unlocking");
        {
            std::unique_lock<std::mutex> lock(tp->task_mutex);
            tp->task_condition.notify_all();
        }
        if (debug>1) RCLCPP_INFO(node->get_logger(), "lt:Unlocked");
        tp->updateStatus(now());

        tp->running = true;
        try {
            // RCLCPP_INFO(node->get_logger(), "Initialising task %s",tp->task->getName().c_str());
            tp->task->doInitialise(tp->tpid,tp->params);
            tp->updateStatus(now());
            if (tp->status != TaskStatus::TASK_INITIALISED) {
                cleanupTask(tp);
                return;
            }
        } catch (const std::exception & e) {
            tp->task->debug("Exception %s",e.what());
            tp->updateStatus(now());
            cleanupTask(tp);
            return ;
        }
        tp->running = true;
        RCLCPP_INFO(node->get_logger(), "Running %s task %d:'%s' at period %f main %d timeout %f periodic %d",tp->foreground?"foreground":"background",tp->task->getRuntimeId(),tp->task->getConfig()->getNameSpace().c_str(),tp->period,(tp==mainThread),tp->task->getTimeout(),tp->task->isPeriodic());

        if (tp->task->isPeriodic()) {
            rclcpp::Rate rate(1. / tp->period);
            if (debug>1) RCLCPP_INFO(node->get_logger(), "Initialisation done");
            while (1) {
                rclcpp::Time tnow = now();
                double t0 = tnow.seconds();
                if (debug > 2) {
                    RCLCPP_INFO(node->get_logger(), "keepAlive %fs", t0 - lastKeepAlive.seconds());
                }
                if (mainThread && (!mainThread->isAnInstanceOf(idle)) && (t0 - lastKeepAlive.seconds() > 1.0)) {
                    tp->task->debug("KEEPALIVE failed");
                    tp->setStatus(TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",now());
                    break;
                }

                if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
                    tp->task->debug("TIMEOUT");
                    tp->setStatus(TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",now());
                    break;
                }

                try {
                    // tp->task->debug("Iterating...");
                    tp->task->doIterate();
                    tp->updateStatus(tnow);
                } catch (const std::exception & e) {
                    tp->task->debug("Exception %s",e.what());
                    tp->setStatus(TaskStatus::TASK_INTERRUPTED, "Interrupted by Exception",tnow);
                    tp->updateStatus(now());
                    cleanupTask(tp);
                    return ;
                }
                if (tp->status != TaskStatus::TASK_RUNNING) {
                    RCLCPP_INFO(node->get_logger(), "Task '%s' not running anymore (%d)",tp->task->getName().c_str(),tp->status);
                    break;
                }
                boost::this_thread::interruption_point();
                rate.sleep();
            }
        } else {
            bool first = true;
            std::unique_lock<std::mutex> lock(tp->aperiodic_task_mutex);
            boost::thread id(&TaskScheduler::runAperiodicTask,this,tp);
            while (1) {
                double t0 = now().seconds();
                if (mainThread && (!mainThread->isAnInstanceOf(idle)) && (t0 - lastKeepAlive.seconds() > 1.0)) {
                    tp->task->debug("KEEPALIVE failed");
                    tp->setStatus(TaskStatus::TASK_INTERRUPTED, "timeout triggered by task keepalive",now());
                    break;
                }
                if ((tp->task->getTimeout() > 0) && ((t0-tstart) > tp->task->getTimeout())) {
                    tp->task->debug("TIMEOUT");
                    tp->setStatus(TaskStatus::TASK_TIMEOUT, "timeout triggered by TaskScheduler",now());
                    break;
                }
                tp->updateStatus(now());
                if (tp->status == TaskStatus::TASK_COMPLETED) {
                    break;
                }
                if (!first && tp->status != TaskStatus::TASK_RUNNING) {
                    RCLCPP_INFO(node->get_logger(), "Task '%s' not running anymore or not reporting itself running in time",tp->task->getName().c_str());
                    break;
                }

                double t1 = now().seconds();
                // Adding a while loop here to account for the fact that we may be running in sim time but the 
                // timed_wait is in real time
                do {
                    double ttimeout = std::max(1e-3,(tp->period - (t1-t0)));
                    tp->aperiodic_task_condition.wait_for(lock,int(ttimeout*1000)*1ms);
                    t1 = now().seconds();
                    // printf("%f / %f\n",t1-t0,tp->period);
                } while ((t1 - t0) < tp->period);
                first = false;
            }
            if (tp->status != TaskStatus::TASK_COMPLETED) {
                id.interrupt();
            }
            id.join();
        }
        // tp->task->debug("Out of the loop");
    } catch (const boost::thread_interrupted & e) {
        RCLCPP_WARN(node->get_logger(), "Task %s interrupted",tp->task->getName().c_str());
        // Ignore, we just want to make sure we get to the next line
        tp->setStatus(TaskStatus::TASK_INTERRUPTED, "Interrupted by Exception",now());
    }
    cleanupTask(tp);
}

void TaskScheduler::terminateTask(std::shared_ptr<ThreadParameters> tp)
{
    if (!tp) return;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Terminating thread %s",tp->task->getName().c_str());
    tp->tid->interrupt();
    tp->tid->join();
    tp->running = false;

    if (debug>1) RCLCPP_INFO(node->get_logger(), "Thread cancelled");
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

void TaskScheduler::cleanupTask(std::shared_ptr<ThreadParameters> tp)
{
    if (tp == NULL) return ;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Cleaning up task %d:%s (%s)",tp->tpid,tp->task->getName().c_str(),tp->foreground?"foreground":"background");
    tp->task->doTerminate();
    if (debug>1) RCLCPP_INFO(node->get_logger(),  "Task '%s' terminated",tp->task->getName().c_str());
    tp->status |= TaskStatus::TASK_TERMINATED;
    tp->statusTime = now();
    tp->statusString = "terminated";
    tp->updateStatus(now());

    std::unique_lock<std::mutex> lock(scheduler_mutex);
    zombieThreads.insert(TaskSetItem(tp->tpid,tp));
    TaskSet::iterator tsit = runningThreads.find(tp->tpid);
    assert(tsit != runningThreads.end());
    runningThreads.erase(tsit);

    if (debug>=3) printTaskSet("Zombies at cleanup",zombieThreads);
    if (debug>=3) printTaskSet("Running at cleanup",runningThreads);


    if (tp->foreground) {
        mainThread.reset();
        if (!tp->task->isAnInstanceOf(idle)) {
            if (debug > 1) {
                RCLCPP_INFO(node->get_logger(),  "Back to idle in %f seconds?", IDLE_TIMEOUT.seconds());
            }
            enqueueAction(now()+IDLE_TIMEOUT,CONDITIONALLY_IDLE,tp);
#if 0
        } else {
            if (debug > 1) {
                // Not sure why this should be weird
                RCLCPP_INFO(node->get_logger(),  "Terminated foreground task (%d) that is not idle (%d). Weird...",tp->task->getDefinition()->getTaskId(), idle->getTaskId());
            }
#endif
        }
    }
    if (debug > 1) {
        RCLCPP_INFO(node->get_logger(),  "Delete task in %f seconds", DELETE_TIMEOUT.seconds());
    }
    enqueueAction(now()+DELETE_TIMEOUT,DELETE_TASK,tp);

    // There is a risk that we trigger the execution of a new task before
    // starting idle here. In addition, we need one condition per task
    tp->task_condition.notify_all();
}

void TaskScheduler::deleteTask(std::shared_ptr<ThreadParameters> tp)
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
    std::unique_lock<std::mutex> lock(scheduler_mutex);
    if (debug>=3) printTaskSet("Zombies when waiting",zombieThreads);
    if (debug>=3) printTaskSet("Running when waiting",runningThreads);

    it = zombieThreads.find(id);
    if (it != zombieThreads.end()) {
        return 0;
    }
    it = runningThreads.find(id);
    if (it == runningThreads.end()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot find reference to task %d",id);
        return -1;
    }
    it->second->task_condition.wait_for(lock,int(timeout*1000)*1ms);
    return 0;
}


void TaskScheduler::printTaskDirectory(bool with_ros) const
{
    unsigned int i = 0;
    TaskDirectory::const_iterator tit;
    if (with_ros) {
        RCLCPP_INFO(node->get_logger(), "Task Directory:");
        for (tit = tasks.begin();tit!=tasks.end();tit++) {
            RCLCPP_INFO(node->get_logger(), "%d -- %s: %s",i,tit->second->getName().c_str(),tit->second->getHelp().c_str());
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
    if (debug>1) RCLCPP_INFO(node->get_logger(), "rci:Locking");
    std::unique_lock<std::mutex> lock(aqMutex);
    if (debug>1) RCLCPP_INFO(node->get_logger(), "rci:Locked");

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

    if (debug>1) RCLCPP_INFO(node->get_logger(), "rci:Signalling");
    aqCond.notify_all();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "rci:Unlocked");
}

TaskScheduler::ThreadAction TaskScheduler::getNextAction()
{
    // Warning: add cancel management
    rclcpp::Time t;
    ThreadAction ta;
    ActionQueue::iterator it;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Locking");
    std::unique_lock<std::mutex> lock(aqMutex);
    if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Locked");
    try {
        // TODO: this must get the next action in time
        if (actionQueue.empty()) {
            if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Cond Wait");
            aqCond.wait(lock);
            if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Locked");
        }
        while (1) {
            t = now();
            it = actionQueue.begin();
            assert(it != actionQueue.end());
            if (runScheduler) {
                if (it->first <= t.seconds()) {
                    ta = it->second;
                    actionQueue.erase(it);
                    if (debug>1) RCLCPP_INFO(node->get_logger(), "Dequeueing action %.3f %s -- %s",it->first,actionString(ta.type),
                            ta.tp?(ta.tp->task->getName().c_str()):"none");
                } else {
                    // wait for the right time or another action to be inserted
                    if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Cond TWait");
                    int dtimeout((it->first-t.seconds())*1000);
                    aqCond.wait_for(lock,dtimeout*1ms);
                    if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Locked");
                    continue;
                }
            } else {
                ta = it->second;
                actionQueue.erase(it);
                if (debug>1) RCLCPP_INFO(node->get_logger(), "Dequeueing action %.3f %s -- %s",it->first,actionString(ta.type),
                        ta.tp?(ta.tp->task->getName().c_str()):"none");
            }
            break;
        }
    } catch (const boost::thread_interrupted & e) {
        ta.type = WAIT_CANCELLED;
        ta.tp.reset();
    }
    if (debug>1) RCLCPP_INFO(node->get_logger(), "gna:Unlocking");
    return ta;
}

void TaskScheduler::enqueueAction(ActionType type,std::shared_ptr<ThreadParameters> tp)
{
    ThreadAction ta;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Locking");
    std::unique_lock<std::mutex> lock(aqMutex);
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Locked");
    // if (!runScheduler) return;
    double when = now().seconds();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Enqueueing actionN %.3f %s -- %s",when,actionString(type),
            tp?(tp->task->getName().c_str()):"none");

    ta.type = type;
    ta.tp = tp;
    actionQueue[when] = ta;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Signalling");
    aqCond.notify_all();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Unlocking");
}

void TaskScheduler::enqueueAction(const rclcpp::Time & when,  ActionType type,std::shared_ptr<ThreadParameters> tp)
{
    ThreadAction ta;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Locking");
    std::unique_lock<std::mutex> lock(aqMutex);
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Locked");
    // if (!runScheduler) return;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Enqueing action@ %.3f %s -- %s",when.seconds(),actionString(type),
            tp?(tp->task->getName().c_str()):"none");

    ta.type = type;
    ta.tp = tp;
    actionQueue[when.seconds()] = ta;
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Signalling");
    aqCond.notify_all();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "ea:Unlocking");
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
        if (debug>1) RCLCPP_INFO(node->get_logger(), "Waiting next action (%d)",(int)(actionQueue.size()));
        if (!runScheduler && actionQueue.empty()) {
            break;
        }

        ThreadAction ta = getNextAction();
        if (debug>1) RCLCPP_INFO(node->get_logger(), "%.3f: got action %s %s",now().seconds(),actionString(ta.type),
                ta.tp?(ta.tp->task->getName().c_str()):"none");
        if (debug>1) RCLCPP_INFO(node->get_logger(), "rsl:Locking");
        switch (ta.type) {
            case START_IDLE_TASK:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "START_IDLE_TASK");
                launchIdleTask();
                break;
            case START_TASK:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "START_TASK %s",ta.tp->task->getName().c_str());
                launchTask(ta.tp);
                break;
            case DELETE_TASK:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "DELETE_TASK %s",ta.tp->task->getName().c_str());
                deleteTask(ta.tp);
                break;
            case CONDITIONALLY_IDLE:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "CONDITIONALLY_IDLE");
                if ((mainThread==NULL) && runScheduler) {
                    // no new task has been created yet, and only this function
                    // can trigger new task creation
                    launchIdleTask();
                }
                break;
            case WAIT_CANCELLED:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "WAIT_CANCELLED");
                break;
            case NO_ACTION:
                if (debug>1) RCLCPP_INFO(node->get_logger(), "NO_ACTION");
                break;
        }
        if (debug>1) RCLCPP_INFO(node->get_logger(), "rsl:Unlocked");
    }
    return 0;
}

int TaskScheduler::startScheduler() 
{
    assert(!runScheduler);
    runScheduler = true;
    enqueueAction(START_IDLE_TASK,std::shared_ptr<ThreadParameters>());
    aqid = boost::thread(&TaskScheduler::runSchedulerLoop,this);
    return 0;
}

int TaskScheduler::stopScheduler()
{
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Stopping scheduler (%d)",runScheduler);
    if (!runScheduler) return 0;
    runScheduler = false;
    enqueueAction(WAIT_CANCELLED,std::shared_ptr<ThreadParameters>());
    aqid.join();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Cleaning-up action queue (%d)",(int)(actionQueue.size()));

    std::unique_lock<std::mutex> lock(aqMutex);
    actionQueue.clear();
    if (debug>1) RCLCPP_INFO(node->get_logger(), "Scheduler stopped");
    return 0;
}


void TaskScheduler::generateTaskList(std::vector<task_manager_msgs::msg::TaskDescription> & tlist) const
{
    TaskDirectory::const_iterator it;
    unsigned int i = 0;
    for (it=tasks.begin();it!=tasks.end();it++,i++) {
        tlist.push_back(it->second->getDescription());
    }
}


void TaskScheduler::generateTaskStatus(std::vector<TaskStatus> & running,
        std::vector<TaskStatus> & zombies) 
{
    TaskSet::const_iterator it;
    std::unique_lock<std::mutex> lock(scheduler_mutex);
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

#if 0
void TaskScheduler::generateTaskListLight(std::vector<task_manager_msgs::TaskDescription> &input,std::vector<task_manager_msgs::TaskDescriptionLight> &output) const
{
    std::vector<task_manager_msgs::TaskDescription> tasklist=input; 
    for (unsigned int i = 0;i<tasklist.size();i++) 
    {
        task_manager_msgs::TaskDescriptionLight current_task;

        current_task.name=tasklist[i].name;
        current_task.description=tasklist[i].description;
        current_task.periodic=tasklist[i].periodic;

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
                    if (debug>1) RCLCPP_INFO(node->get_logger(), "TYPE NOT LEGAL");
                }
                current_task.parameters.push_back(current_parameter);
            }



        }
    }


output.push_back(current_task);


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
    std::shared_ptr<ThreadParameters> tp(new ThreadParameters(statusPub, this, st, defaultPeriod ));
    tp->foreground = false;
    id=launchTask(tp);
}
#endif

int TaskScheduler::getStatus(unsigned int &taskid) {
    TaskSet::const_iterator it;
    for (it=runningThreads.begin();it!=runningThreads.end();it++) {
        if (it->first==taskid) {
            return it->second->getRosStatus().status;
        }
    }
    for (it=zombieThreads.begin();it!=zombieThreads.end();it++) {
        if (it->first==taskid) {
            return it->second->getRosStatus().status;
        }
    }
    return -1;
}

int TaskScheduler::terminateTask(unsigned int &taskid)
{
    std::unique_lock<std::mutex> lock(scheduler_mutex);
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
    lastKeepAlive = now();
}


