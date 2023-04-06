import time

from task_manager_msgs.srv import StartTask
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
import rclpy
# from rclpy.executors import MultiThreadedExecutor

from TaskClient import TaskClient

def status(tc):
    tc.printTaskStatus()


def index(tc):
    print("Known tasks summary:")
    for t in tc.tasklist.values():
        print("  %-16s: %s" % (t.name,t.help))
    print( "Tasks name can be used as functions, e.g. Wait(duration=1.0)")
    print( "Use help(Task) to get help on a specific task, e.g. help(Wait)")
    print( "Use Ctrl-C to stop the keep-alive thread and kill all tasks")
    print( "Type status() to display the status of currently running tasks")
    print( "Type index() to display this summary")


def main(args=None):
    rclpy.init(args=args)

    server_node = "/turtlesim_tasks"
    tc = TaskClient(server_node, 0.1)
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(tc)
    print("Task client created")
    tc.verbose = 1
    index(tc)
    status(tc)
    tc.Clear()
    tc.GoTo(goal_x=9,goal_y=9)
    tc.GoTo(goal_x=2,goal_y=2)
    tc.GoTo(goal_x=5,goal_y=5)

    # del(tc)
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
