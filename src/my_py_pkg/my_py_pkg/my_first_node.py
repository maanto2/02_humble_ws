#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello ROS2 humble')

        self.i = 0
        #create_timer method creates a timer that calls a callback function at a specified rate
        self.create_timer(5, self.timer_callback)


    #method names timer_callback, this method will be called every 1 second
    def timer_callback(self):
        self.get_logger().info('Hello ROS2 %d' % self.i)
        self.i += 1


# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()