#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed 


class LedPanelNode(Node):
    def __init__(self):
        super().__init__('led_panel_state_publisher')
        self.pub = self.create_publisher(LedPanelState, 'led_panel_state', 10)
        self.create_timer(1, self.publish_news) # this will call the publish_news method every 5 seconds
        
        # Create a service server
        self.srv = self.create_service(SetLed, 'set_led', self.handle_led_state)

    def handle_led_state(self, request, response):
        self.get_logger().info('Incoming request: a=%d b=%s' % (request.led, request.state))
        if request.state == "on":
            response.status = True
            response.ack = "leds turned on"
        elif request.state == "off ":
            response.status = True
            response.ack = "leds turned off"
        else:
            response.status = False
            response.ack = "Invalid state"
        return response


    def publish_news(self):
        msg = LedPanelState()
        msg.temperature = 25.0
        msg.battery_charged = True
        msg.debug_message = "No LEDs turned on yet"
        self.pub.publish(msg)



# this is the main function that will be called when we run the node    
def main(args=None):
    rclpy.init(args=args)

    node = LedPanelNode()
    rclpy.spin(node) # this will block the program until the node is shutdown
    rclpy.shutdown() # this will shutdown the node


if __name__ == '__main__':
    main()