import sys

from task_manager_msgs.srv import StartTask
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
import rclpy
from rclpy.node import Node

def P(name,value):
    P = Parameter()
    P.name = name
    if type(value)==int:
        P.value.type = ParameterType.PARAMETER_INTEGER
        P.value.integer_value = value
    elif type(value)==float:
        P.value.type = ParameterType.PARAMETER_DOUBLE
        P.value.double_value = value
    elif type(value)==bool:
        P.value.type = ParameterType.PARAMETER_BOOL
        P.value.bool_value = value
    elif type(value)==str:
        P.value.type = ParameterType.PARAMETER_STRING
        P.value.string_value = value
    #ignoring arrays for now
    return P

class StartTaskAsync(Node):

    def __init__(self):
        super().__init__('start_task_async')
        self.cli = self.create_client(StartTask, '/turtlesim_tasks/start_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StartTask.Request()


    def send_request(self, name):
        if name == "Clear":
            self.req.name = "Clear"
            self.req.config = []
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        elif name == "SetPen":
            self.req.name = "SetPen"
            self.req.config = [P("on",True),P("r",255),P("g",0),P("b",0),P("width",1)]
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()



def main(args=None):
    rclpy.init(args=args)

    start_client = StartTaskAsync()
    response = start_client.send_request(sys.argv[1])
    start_client.get_logger().info( 'Result of start_task: %s' % str(response))
    start_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
