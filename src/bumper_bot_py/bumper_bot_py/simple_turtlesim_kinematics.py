from rclpy.node import Node
import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class SimpleTurtleSimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtle_sim_kinematics")


        # Create a subscriber for the turtle1 pose
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,"/turtle1/pose",self.turtle1_pose_callback, 10 )
        
        
        #create a subscriber for the turtle2 pose
        self.turtle2_pose_sub_ = self.create_subscription(
            Pose,"/turtle2/pose", self.turtle2_pose_callback,10 )


        self.last_turtle1_pose = Pose()
        self.last_turtle2_pose = Pose()

    
    def turtle1_pose_callback(self, msg: Pose):
        # Store the last pose of turtle1
        self.last_turtle1_pose = msg

    def turtle2_pose_callback(self, msg: Pose):
        # Store the last pose of turtle1
        self.last_turtle2_pose = msg

        Tx = self.last_turtle2_pose.x - self.last_turtle1_pose.x
        Ty = self.last_turtle2_pose.y - self.last_turtle1_pose.y

        # Calculate the angle difference
        theta = self.last_turtle1_pose.theta - self.last_turtle2_pose.theta
        theta_deg  = theta * 180 / 3.14

        self.get_logger().info(""" \n
               Translation from turtle1 to turtle2: \n 
               Tx : %f  \n 
               Ty : %f  \n
               Rotation matrix t1->t2 (rad) : %f \n
               Rotation matrix t1->t2 (deg) : %f \n 
               |R11     R12| : |%f       %f| \n
               |R21     R22| : |%f       %f| \n """ % (Tx, Ty, theta, theta_deg , math.cos(theta) , -math.sin(theta), 
                                                                                  math.sin(theta) , math.cos(theta)))

def main(args=None):
    rclpy.init()
    node = SimpleTurtleSimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



        
