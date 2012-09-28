uint8 TASK_NEWBORN = 0 
uint8 TASK_CONFIGURED = 1
uint8 TASK_INITIALISED = 2
uint8 TASK_RUNNING = 3
uint8 TASK_COMPLETED = 4
uint8 TASK_TERMINATED = 5
uint8 TASK_INTERRUPTED = 6
uint8 TASK_FAILED = 7
uint8 TASK_TIMEOUT = 8
uint8 TASK_CONFIGURATION_FAILED = 9
uint8 TASK_INITIALISATION_FAILED = 10

uint32 id
string name
uint8 status
string status_string
time status_time
dynamic_reconfigure/Config plist
