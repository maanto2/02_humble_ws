#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class AddIntsClientNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        self.server = self.create_client(AddTwoInts, 'add_two_ints')

    
    def send_request(self, a, b):
        self.get_logger().info('Sending request')
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.server.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        print(response.sum)
# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    if (len(sys.argv) != 3):
        print("Usage: ros2 run my_py_pkg add_ints_client_node <a> <b>")
        sys.exit(1)
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    node = AddIntsClientNode()
    node.send_request(a, b)
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()