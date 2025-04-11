#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#create a node with the name 'my_first_node', inside the node we will write the code to publish and subscribe to topics
# this node sits inside the 'my_py_pkg' package
#so a pakage is a collection of nodes or a single node

class RobotStation(Node):
    def __init__(self):
        super().__init__('robot_station_node')
        self.pub = self.create_publisher(String, 'robot_news', 10)
        self.create_timer(5, self.publish_news) # this will call the publish_news method every 5 seconds

        
        self.sub = self.create_subscription(String, 'robot_news', self.listener_callback, 10)

    def publish_news(self):
        msg = String()
        msg.data = 'Hello from the Robot Station!'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        #create_timer method creates a timer that calls a callback function at a specified rate
        #self.create_timer(5, self.timer_callback)
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # this method will be called when a message is received on the topic 'robot_news'
        # it will print the message to the console


# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = RobotStation()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()