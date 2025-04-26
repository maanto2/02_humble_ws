#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class SimpleController(Node):
    """
    A simple controller class that allows you to set and get parameters.
    """

    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.17)


        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value


        self.get_logger().info(f'Wheel radius: {self.wheel_radius} \n'
                            f'Wheel separation: {self.wheel_separation}')
        

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, 'simple_velocity_controller/commands', 10)
        self.vel_subscriber = self.create_subscription(Twist, 'bumperbot_controller/cmd_vel', self.vel_callback, 10)

        # Conversion matrix for wheel speeds
        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2], 
                                           [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
                                           ])  


    
    def vel_callback(self, msg):

        robot_speed = np.array([msg.twist.linear.x],
                               [msg.twist.angular.z])

        # Calculate the wheel velocities
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        wheel_speed_cmd = Float64MultiArray()
        wheel_speed_cmd.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        # Publish the wheel commands
        self.wheel_cmd_pub.publish(wheel_speed_cmd)


def main(args=None):
    rclpy.init(args = args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

