#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"



class SimpleTFKinematics : public rclcpp::Node
{
  public:
    SimpleTFKinematics() : Node("simple_tf_kinematics")
    {
      // Create a static transform broadcaster
      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      //std::shared_ptr<tf2::StaticTransformBroadcaster> static_broadcaster_ (new tf2::StaticTransformBroadcaster(this));
      // new is returning a raw pointer of type tf2::StaticTransformBroadcaster which is pointing to SimpleTFKinematics class

      dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Create a transform message
      geometry_msgs::msg::TransformStamped static_transform_stamped;


      static_transform_stamped.header.stamp = this->now();  // get_clock()->now();

      static_transform_stamped.header.frame_id = "bumperbot_base";
      static_transform_stamped.child_frame_id = "bumperbot_top";
      static_transform_stamped.transform.translation.x = 0.5;
      static_transform_stamped.transform.translation.y = 0.0;
      static_transform_stamped.transform.translation.z = 0.0;
      static_transform_stamped.transform.rotation.x = 0.0;
      static_transform_stamped.transform.rotation.y = 0.0;
      static_transform_stamped.transform.rotation.z = 0.0;
      static_transform_stamped.transform.rotation.w = 1.0;

      // Send the static transform
      static_broadcaster_->sendTransform(static_transform_stamped);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimpleTFKinematics::sendDynamicTransform, this));
      
    }

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;

        float x_last_ = 0.0;
        float x_increment_ = 0.1;

        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

        void sendDynamicTransform()
        {
            // Create a transform message
            dynamic_transform_stamped.header.stamp = this->now();  // get_clock()->now();

            dynamic_transform_stamped.header.frame_id = "odom";
            dynamic_transform_stamped.child_frame_id = "bumperbot_base";
            dynamic_transform_stamped.transform.translation.x = x_last_ + x_increment_;
            dynamic_transform_stamped.transform.translation.y = 0.0;
            dynamic_transform_stamped.transform.translation.z = 0.0;
            dynamic_transform_stamped.transform.rotation.x = 0.0;
            dynamic_transform_stamped.transform.rotation.y = 0.0;
            dynamic_transform_stamped.transform.rotation.z = 0.0;
            dynamic_transform_stamped.transform.rotation.w = 1.0;

            // Send the static transform
            dynamic_broadcaster_->sendTransform(dynamic_transform_stamped);

            x_last_= dynamic_transform_stamped.transform.translation.x;

        }
    };


int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTFKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}