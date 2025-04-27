import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')
        
        # Create a StaticTransformBroadcaster Object
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Create a TransformBroadcaster Object
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # create a TransformStamped object to receive the transform
        self.static_tranform_stamped = TransformStamped()
        self.dynamic_tranform_stamped = TransformStamped()

        self.static_tranform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_tranform_stamped.header.frame_id = 'bumperbot_base'
        self.static_tranform_stamped.child_frame_id = 'bumperbot_top'
        # Set the translation and rotation
        self.static_tranform_stamped.transform.translation.x = 0.0
        self.static_tranform_stamped.transform.translation.y = 0.0  
        self.static_tranform_stamped.transform.translation.z = 0.3
        self.static_tranform_stamped.transform.rotation.x = 0.0
        self.static_tranform_stamped.transform.rotation.y = 0.0
        self.static_tranform_stamped.transform.rotation.z = 0.0
        self.static_tranform_stamped.transform.rotation.w = 1.0
        

        self.static_tf_broadcaster.sendTransform(self.static_tranform_stamped)
        self.get_logger().info('Static transform broadcaster started')


        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.x_increment_ = 0.05
        self.x_last_ = 0.0

        self.w_increment_ = np.pi / 180.0
        self.w_last_ = 0.0


    

    def timer_callback(self):

        # Set the header stamp
        self.dynamic_tranform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tranform_stamped.header.frame_id = 'odom'
        self.dynamic_tranform_stamped.child_frame_id = 'bumperbot_base'

        # Set the translation and rotation
        self.dynamic_tranform_stamped.transform.translation.x = self.x_last_ + self.x_increment_
        self.dynamic_tranform_stamped.transform.translation.y = 0.0  
        self.dynamic_tranform_stamped.transform.translation.z = 0.0
        self.dynamic_tranform_stamped.transform.rotation.x = 0.0
        self.dynamic_tranform_stamped.transform.rotation.y = 0.0
        self.dynamic_tranform_stamped.transform.rotation.z = 0.0
        self.dynamic_tranform_stamped.transform.rotation.w = self.w_last_ + self.w_increment_

        # Send the transform
        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_tranform_stamped)
        self.get_logger().info('Dynamic transform broadcaster started')

        self.x_last_ = self.dynamic_tranform_stamped.transform.translation.x
        self.w_last_ = self.dynamic_tranform_stamped.transform.rotation.w


def main(args=None):
    rclpy.init(args=args)
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


