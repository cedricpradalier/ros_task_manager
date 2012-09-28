
#include "TaskParameters.h"

const char message[] = 
"message_length: 356\n"
"  command   :    run_task   \n"
"task_name: hover\n"
"task_timeout: 30\n"
"    \t \n"
"  hover_height    :    0.45   \n"
;


int main() {
	TaskParameters tp;
	printf("Loading from string\n");
	if (tp.loadFromString(message)) {
		return -1;
	}
	printf("message_length: %ld\n",tp["message_length"].asLong());
	printf("command: %s\n",tp["command"].asChar());
	printf("task_name: %s\n",tp["task_name"].asChar());
	printf("task_timeout: %ld\n",tp["task_timeout"].asLong());
	printf("hover_height: %f\n",tp["hover_height"].asDouble());

	printf("Saving to file\n");
	if (tp.saveToFile("message.txt")) {
		return -1;
	}

	TaskParameters tf;
	printf("Loading from file\n");
	if (tf.loadFromFile("message.txt")) {
		return -1;
	}
	printf("message_length: %ld\n",tf["message_length"].asLong());
	printf("command: %s\n",tf["command"].asChar());
	printf("task_name: %s\n",tf["task_name"].asChar());
	printf("task_timeout: %ld\n",tf["task_timeout"].asLong());
	printf("hover_height: %f\n",tf["hover_height"].asDouble());

	return 0;
}
