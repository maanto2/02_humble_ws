#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"



class SimpleTFKinematics : public rclcpp::Node
{
  public:
    SimpleTFKinematics() : Node("simple_tf_kinematics")
    {
      // Create a static transform broadcaster
      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      //std::shared_ptr<tf2::StaticTransformBroadcaster> static_broadcaster_ (new tf2::StaticTransformBroadcaster(this));
      // new is returning a raw pointer of type tf2::StaticTransformBroadcaster which is pointing to SimpleTFKinematics class

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
    }

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    };


int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTFKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}