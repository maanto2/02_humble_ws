#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_status_publisher')
        self.pub = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.create_timer(5, self.publish_news) # this will call the publish_news method every 5 seconds


    def publish_news(self):
        msg = HardwareStatus()
        msg.temperature = 25.0
        msg.are_motors_ready = True
        msg.debug_message = "All systems operational"


        self.pub.publish(msg)



# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = HardwareNode()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()