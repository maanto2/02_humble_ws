import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')
        
        # Create a StaticTransformBroadcaster Object
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # create a TransformStamped object to receive the transform
        self.static_tranform_stamped = TransformStamped()

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


def main(args=None):
    rclpy.init(args=args)
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


