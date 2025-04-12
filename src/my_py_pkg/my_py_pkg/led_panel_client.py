#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed 
import sys
import random

#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class LedPanelClient(Node):
    def __init__(self):
        super().__init__('led_panel_client_node')

        self.client = self.create_client(SetLed, 'set_led')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self):
        self.get_logger().info('Sending request')
        request = SetLed.Request()
        k = random.randint(0, 1) 
        request.led = k
        if k == 0:
            request.state =  "off"
        else:
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