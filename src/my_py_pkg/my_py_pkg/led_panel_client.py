#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed 
import sys

#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class LedPanelClient(Node):
    def __init__(self):
        super().__init__('led_panel_client_node')

        self.client = self.create_client(SetLed, 'set_led')

    
    def send_request(self):
        self.get_logger().info('Sending request')
        request = SetLed.Request()
        request.led = 11
        request.state =  "on"

        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        self.get_logger().info('accepting response')
        response = future.result()
        print(response.status)
        print(response.ack)
# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = LedPanelClient()
    node.send_request()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()