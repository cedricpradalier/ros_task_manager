#!/usr/bin/python
# ROS specific imports
import sys

from task_manager_msgs.srv import GetTaskList
import rclpy
from rclpy.node import Node


server_node="turtlesim_tasks"
if len(sys.argv)>1:
    server_node=sys.argv[1]

class GetTaskListAsync(Node):

    def __init__(self, server):
        super().__init__('get_task_list_async')
        self.cli = self.create_client(GetTaskList, '/%s/get_all_tasks'%server)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetTaskList.Request()


    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



def main(args=None):
    rclpy.init(args=args)

    get_client = GetTaskListAsync(server_node)
    response = get_client.send_request()
    start_client.get_logger().info( 'Result of start_task:\n %s' % str(response))
    start_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

