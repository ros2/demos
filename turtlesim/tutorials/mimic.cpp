#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/Pose.hpp>
#include <geometry_msgs/msg/Twist.hpp>

class Mimic : public rclcpp::Node
  : Node("turtle_mimic")
{
 public:
  explicit Mimic();

 private:
  // void poseCallback(const turtlesim::PoseConstPtr& pose);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

Mimic::Mimic()
{
  // ros::NodeHandle input_nh("input");
  // ros::NodeHandle output_nh("output");

  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", rmw_qos_profile_default);

  auto pose_call_back = 
    [this](const turtlesim::msg::Pose::SharedPtr pose) -> void
    {
      auto twist = std::make_shared<geometry_msgs::msg::Twist>();
      twist->angular.z = pose->angular_velocity;
      twist->linear.x = pose->linear_velocity;
      twist_pub_->publish(twist);
    };

  pose_sub_ = create_subscription<turtlesim::msg::Pose>("input/pose", pose_call_back);
}

// void Mimic::poseCallback(const turtlesim::PoseConstPtr& pose)
// {
  // geometry_msgs::Twist twist;
  // twist.angular.z = pose->angular_velocity;
  // twist.linear.x = pose->linear_velocity;
  // twist_pub_.publish(twist);
// }

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Mimic>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

