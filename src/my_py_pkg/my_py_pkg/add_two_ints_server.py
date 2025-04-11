#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class AddIntsServerNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        self.server = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)


    #method names timer_callback, this method will be called every 1 second
    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request: a=%d b=%d' % (request.a, request.b))
        response.sum = request.a + request.b
        self.get_logger().info('Outgoing response: sum=%d' % (response.sum))
        return response


# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = AddIntsServerNode()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()